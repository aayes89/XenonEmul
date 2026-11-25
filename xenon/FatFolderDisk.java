package pcp.xenon;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Disco USB virtual: presenta una carpeta del equipo como un disco con MBR y
 * una partición FAT32 (con nombres largos). Java puro, sin librerías externas.
 *
 * - El contenido de los ficheros se lee a demanda de la carpeta real.
 * - Las escrituras del guest se guardan en una capa en memoria (overlay) y,
 *   con {@link #flushToHost()}, los ficheros nuevos o modificados se exportan
 *   a la carpeta real (nunca se borra nada de la carpeta).
 */
public class FatFolderDisk {

    public static final int SECTOR = 512;
    private static final int PART_START = 2048;      // LBA de inicio de la partición
    private static final int RESERVED = 32;
    private static final int NUM_FATS = 2;
    private static final int MIN_CLUSTERS = 66000;   // FAT32 exige >= 65525 clusters

    private static final class Node {
        String name;
        boolean dir;
        File file;
        long size;
        int start;            // primer cluster (0 si vacío)
        int clusters;
        Node parent;
        List<Node> children = new ArrayList<>();
        byte[] dirData;       // contenido de directorio ya generado
        String shortName;     // 11 caracteres
    }

    private static final class Extent {
        final int start;
        final int count;
        final Node node;

        Extent(int start, int count, Node node) {
            this.start = start;
            this.count = count;
            this.node = node;
        }
    }

    private final Path folder;
    private final int spc;                 // sectores por cluster
    private final int clusterBytes;
    private final int totalClusters;
    private final int fatSectors;
    private final int dataStart;           // sector relativo a la partición
    private final long volumeSectors;
    private final long diskSectors;
    private final int[] fat;
    private final List<Extent> extents = new ArrayList<>();
    private final Map<Long, byte[]> overlay = new HashMap<>();
    private final Map<String, long[]> original = new HashMap<>(); // ruta -> {inicio, tamaño}
    private final Map<File, RandomAccessFile> open = new LinkedHashMap<File, RandomAccessFile>(16, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<File, RandomAccessFile> e) {
            if (size() > 8) {
                try {
                    e.getValue().close();
                } catch (IOException ignored) {
                }
                return true;
            }
            return false;
        }
    };
    private volatile boolean dirty = false;
    private volatile long lastWriteMs = 0;
    private final byte[] mbr = new byte[SECTOR];
    private final byte[] vbr = new byte[SECTOR];
    private final byte[] fsinfo = new byte[SECTOR];
    private int nextFreeHint;

    public FatFolderDisk(Path folder) throws IOException {
        this.folder = folder;
        Node root = new Node();
        root.name = "";
        root.dir = true;
        scan(root, folder.toFile(), 0);

        // 1) tamaño de cluster y número de clusters
        long content = totalContent(root);
        int s = 8;
        while (s < 128 && content / ((long) s * SECTOR) > 3_000_000L) {
            s *= 2;
        }
        spc = s;
        clusterBytes = spc * SECTOR;
        long needed = clustersNeeded(root);
        long total = Math.max(MIN_CLUSTERS, needed + needed / 4 + 4096);
        if (total > 0x0FFFFFF0L) {
            throw new IOException("La carpeta es demasiado grande para FAT32");
        }
        totalClusters = (int) total;
        fat = new int[totalClusters + 2];
        fatSectors = ((totalClusters + 2) * 4 + SECTOR - 1) / SECTOR;
        dataStart = RESERVED + NUM_FATS * fatSectors;
        volumeSectors = (long) dataStart + (long) totalClusters * spc;
        diskSectors = PART_START + volumeSectors;

        // 2) asignación contigua de clusters (raíz = cluster 2)
        int[] next = {2};
        allocate(root, next);
        nextFreeHint = next[0];
        buildDirectories(root);
        recordOriginal(root, "");
        buildBootSectors();
    }

    // ------------------------------------------------------------------ escaneo
    private void scan(Node dir, File f, int depth) {
        File[] list = f.listFiles();
        if (list == null) {
            return;
        }
        Arrays.sort(list, (a, b) -> a.getName().compareToIgnoreCase(b.getName()));
        for (File c : list) {
            String n = c.getName();
            if (n.length() > 255) {
                continue;
            }
            Node node = new Node();
            node.name = n;
            node.parent = dir;
            if (c.isDirectory()) {
                if (depth > 16) {
                    continue;
                }
                node.dir = true;
                scan(node, c, depth + 1);
            } else if (c.isFile()) {
                long len = c.length();
                if (len > 0xFFFFFFFFL) {
                    continue;
                }
                node.file = c;
                node.size = len;
            } else {
                continue;
            }
            dir.children.add(node);
        }
    }

    private long totalContent(Node n) {
        long t = 0;
        for (Node c : n.children) {
            t += c.dir ? totalContent(c) : c.size;
        }
        return t;
    }

    private long clustersNeeded(Node n) {
        long cb = clusterBytes;
        long t = 1; // el propio directorio (mínimo 1 cluster; se ajusta luego)
        t += ((long) (n.children.size() * 22 + 64) / cb);
        for (Node c : n.children) {
            if (c.dir) {
                t += clustersNeeded(c);
            } else {
                t += (c.size + cb - 1) / cb;
            }
        }
        return t;
    }

    private int dirClusters(Node d) {
        int bytes = dirSize(d);
        return Math.max(1, (bytes + clusterBytes - 1) / clusterBytes);
    }

    private int dirSize(Node d) {
        int entries = d.parent == null ? 0 : 2;
        for (Node c : d.children) {
            entries += 1 + (c.name.length() + 12) / 13;
        }
        return (entries + 1) * 32;
    }

    private void allocate(Node n, int[] next) {
        n.clusters = n.dir ? dirClusters(n) : (int) ((n.size + clusterBytes - 1) / clusterBytes);
        if (n.clusters > 0) {
            n.start = next[0];
            for (int i = 0; i < n.clusters; i++) {
                fat[n.start + i] = (i == n.clusters - 1) ? 0x0FFFFFFF : n.start + i + 1;
            }
            extents.add(new Extent(n.start, n.clusters, n));
            next[0] += n.clusters;
        }
        for (Node c : n.children) {
            allocate(c, next);
        }
    }

    // --------------------------------------------------------- directorios FAT
    private void buildDirectories(Node n) {
        if (n.dir) {
            Set<String> used = new HashSet<>();
            byte[] out = new byte[n.clusters * clusterBytes];
            int pos = 0;
            if (n.parent != null) {
                putEntry(out, pos, ".          ", 0x10, n.start, 0, n.file);
                pos += 32;
                int p = n.parent.parent == null ? 0 : n.parent.start;
                putEntry(out, pos, "..         ", 0x10, p, 0, n.file);
                pos += 32;
            }
            for (Node c : n.children) {
                c.shortName = makeShortName(c.name, used);
                String lfn = needsLfn(c.name, c.shortName) ? c.name : null;
                if (lfn != null) {
                    pos = putLfn(out, pos, lfn, c.shortName);
                }
                putEntry(out, pos, c.shortName, c.dir ? 0x10 : 0x20, c.start, c.dir ? 0 : (int) c.size, c.file);
                pos += 32;
            }
            n.dirData = out;
        }
        for (Node c : n.children) {
            buildDirectories(c);
        }
    }

    private static boolean needsLfn(String name, String shortName) {
        String sn = shortName.substring(0, 8).trim();
        String se = shortName.substring(8).trim();
        String rebuilt = se.isEmpty() ? sn : sn + "." + se;
        return !rebuilt.equals(name);
    }

    private static String makeShortName(String name, Set<String> used) {
        String up = name.toUpperCase();
        int dot = up.lastIndexOf('.');
        String rawBase = dot > 0 ? up.substring(0, dot) : up;
        String rawExt = dot > 0 ? up.substring(dot + 1) : "";
        String base = clean(rawBase);
        String ext = clean(rawExt);
        boolean valid83 = dot != 0 && rawBase.length() <= 8 && rawExt.length() <= 3
                && base.equals(rawBase.replace(" ", "")) && rawBase.indexOf('.') < 0 && rawBase.indexOf(' ') < 0
                && ext.equals(rawExt) && !rawBase.isEmpty();
        if (base.isEmpty()) {
            base = "_";
        }
        if (ext.length() > 3) {
            ext = ext.substring(0, 3);
        }
        String candidate;
        if (valid83) {
            candidate = pad(base, 8) + pad(ext, 3);
            if (used.add(candidate)) {
                return candidate;
            }
        }
        String b6 = base.length() > 6 ? base.substring(0, 6) : base;
        for (int i = 1; i < 1000000; i++) {
            String tail = "~" + i;
            String b = b6.length() + tail.length() > 8 ? b6.substring(0, 8 - tail.length()) : b6;
            candidate = pad(b + tail, 8) + pad(ext, 3);
            if (used.add(candidate)) {
                return candidate;
            }
        }
        throw new IllegalStateException("No se pudo generar nombre corto");
    }

    private static String clean(String s) {
        StringBuilder sb = new StringBuilder();
        for (char ch : s.toCharArray()) {
            if (ch == ' ' || ch == '.') {
                continue;
            }
            if ("\"*+,/:;<=>?[\\]|".indexOf(ch) >= 0 || ch < 0x20 || ch > 0x7E) {
                sb.append('_');
            } else {
                sb.append(ch);
            }
        }
        return sb.toString();
    }

    private static String pad(String s, int n) {
        StringBuilder sb = new StringBuilder(s);
        while (sb.length() < n) {
            sb.append(' ');
        }
        return sb.toString();
    }

    private static int lfnChecksum(String shortName) {
        int sum = 0;
        byte[] b = shortName.getBytes(StandardCharsets.ISO_8859_1);
        for (int i = 0; i < 11; i++) {
            sum = (((sum & 1) << 7) | ((sum & 0xFF) >> 1)) + (b[i] & 0xFF);
            sum &= 0xFF;
        }
        return sum;
    }

    private int putLfn(byte[] out, int pos, String name, String shortName) {
        int chk = lfnChecksum(shortName);
        int n = (name.length() + 12) / 13;
        char[] chars = new char[n * 13];
        Arrays.fill(chars, (char) 0xFFFF);
        for (int i = 0; i < name.length(); i++) {
            chars[i] = name.charAt(i);
        }
        if (name.length() < chars.length) {
            chars[name.length()] = 0;
        }
        int[] offs = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
        for (int seq = n; seq >= 1; seq--) {
            int base = pos;
            out[base] = (byte) (seq | (seq == n ? 0x40 : 0));
            out[base + 11] = 0x0F;
            out[base + 13] = (byte) chk;
            for (int k = 0; k < 13; k++) {
                char ch = chars[(seq - 1) * 13 + k];
                out[base + offs[k]] = (byte) ch;
                out[base + offs[k] + 1] = (byte) (ch >> 8);
            }
            pos += 32;
        }
        return pos;
    }

    private void putEntry(byte[] out, int pos, String sn, int attr, int cluster, int size, File f) {
        byte[] b = sn.getBytes(StandardCharsets.ISO_8859_1);
        System.arraycopy(b, 0, out, pos, 11);
        out[pos + 11] = (byte) attr;
        long ms = f != null ? f.lastModified() : System.currentTimeMillis();
        java.util.Calendar c = java.util.Calendar.getInstance();
        c.setTimeInMillis(ms);
        int year = Math.max(1980, c.get(java.util.Calendar.YEAR));
        int date = ((year - 1980) << 9) | ((c.get(java.util.Calendar.MONTH) + 1) << 5) | c.get(java.util.Calendar.DAY_OF_MONTH);
        int time = (c.get(java.util.Calendar.HOUR_OF_DAY) << 11) | (c.get(java.util.Calendar.MINUTE) << 5)
                | (c.get(java.util.Calendar.SECOND) / 2);
        le16(out, pos + 14, time);
        le16(out, pos + 16, date);
        le16(out, pos + 18, date);
        le16(out, pos + 20, cluster >>> 16);
        le16(out, pos + 22, time);
        le16(out, pos + 24, date);
        le16(out, pos + 26, cluster & 0xFFFF);
        le32(out, pos + 28, size);
    }

    private void recordOriginal(Node n, String path) {
        for (Node c : n.children) {
            String p = path + "/" + c.name;
            original.put(p, new long[]{c.start, c.dir ? -1 : c.size});
            if (c.dir) {
                recordOriginal(c, p);
            }
        }
    }

    // --------------------------------------------------------- sectores de arranque
    private void buildBootSectors() {
        // MBR con una partición FAT32 LBA (0x0C)
        byte[] e = mbr;
        e[446] = (byte) 0x80;
        e[447] = 0x00;
        e[448] = 0x21;
        e[449] = 0x00;
        e[450] = 0x0C;
        e[451] = (byte) 0xFE;
        e[452] = (byte) 0xFF;
        e[453] = (byte) 0xFF;
        le32(e, 454, PART_START);
        le32(e, 458, (int) volumeSectors);
        le32(e, 440, 0x58454E4F); // firma de disco
        e[510] = 0x55;
        e[511] = (byte) 0xAA;

        byte[] v = vbr;
        v[0] = (byte) 0xEB;
        v[1] = 0x58;
        v[2] = (byte) 0x90;
        System.arraycopy("MSDOS5.0".getBytes(StandardCharsets.ISO_8859_1), 0, v, 3, 8);
        le16(v, 11, SECTOR);
        v[13] = (byte) spc;
        le16(v, 14, RESERVED);
        v[16] = NUM_FATS;
        le16(v, 17, 0);
        le16(v, 19, 0);
        v[21] = (byte) 0xF8;
        le16(v, 22, 0);
        le16(v, 24, 63);
        le16(v, 26, 255);
        le32(v, 28, PART_START);
        le32(v, 32, (int) volumeSectors);
        le32(v, 36, fatSectors);
        le16(v, 40, 0);
        le16(v, 42, 0);
        le32(v, 44, 2);
        le16(v, 48, 1);
        le16(v, 50, 6);
        v[64] = (byte) 0x80;
        v[66] = 0x29;
        le32(v, 67, 0x1234ABCD);
        System.arraycopy("XENON USB  ".getBytes(StandardCharsets.ISO_8859_1), 0, v, 71, 11);
        System.arraycopy("FAT32   ".getBytes(StandardCharsets.ISO_8859_1), 0, v, 82, 8);
        v[510] = 0x55;
        v[511] = (byte) 0xAA;

        byte[] f = fsinfo;
        le32(f, 0, 0x41615252);
        le32(f, 484, 0x61417272);
        le32(f, 488, totalClusters - (nextFreeHint - 2));
        le32(f, 492, nextFreeHint);
        le32(f, 508, 0xAA550000);
    }

    // ------------------------------------------------------------------ lectura
    public long totalSectors() {
        return diskSectors;
    }

    public synchronized void readSector(long lba, byte[] dst, int off) {
        byte[] ov = overlay.get(lba);
        if (ov != null) {
            System.arraycopy(ov, 0, dst, off, SECTOR);
            return;
        }
        Arrays.fill(dst, off, off + SECTOR, (byte) 0);
        if (lba < 0 || lba >= diskSectors) {
            return;
        }
        if (lba == 0) {
            System.arraycopy(mbr, 0, dst, off, SECTOR);
            return;
        }
        long rel = lba - PART_START;
        if (rel < 0) {
            return;
        }
        if (rel == 0 || rel == 6) {
            System.arraycopy(vbr, 0, dst, off, SECTOR);
        } else if (rel == 1 || rel == 7) {
            System.arraycopy(fsinfo, 0, dst, off, SECTOR);
        } else if (rel >= RESERVED && rel < dataStart) {
            int fs = (int) ((rel - RESERVED) % fatSectors);
            int first = fs * (SECTOR / 4);
            for (int i = 0; i < SECTOR / 4 && first + i < fat.length; i++) {
                le32(dst, off + i * 4, fat[first + i]);
            }
            if (fs == 0) {
                le32(dst, off, 0x0FFFFFF8);
                le32(dst, off + 4, 0x0FFFFFFF);
            }
        } else if (rel >= dataStart) {
            long dataRel = rel - dataStart;
            int cluster = (int) (dataRel / spc) + 2;
            int inCluster = (int) (dataRel % spc);
            Extent ex = findExtent(cluster);
            if (ex != null) {
                long byteOff = (long) (cluster - ex.start) * clusterBytes + (long) inCluster * SECTOR;
                Node n = ex.node;
                if (n.dir) {
                    System.arraycopy(n.dirData, (int) byteOff, dst, off, SECTOR);
                } else if (byteOff < n.size) {
                    readFile(n.file, byteOff, dst, off, (int) Math.min(SECTOR, n.size - byteOff));
                }
            }
        }
    }

    private Extent findExtent(int cluster) {
        int lo = 0, hi = extents.size() - 1;
        while (lo <= hi) {
            int mid = (lo + hi) >>> 1;
            Extent e = extents.get(mid);
            if (cluster < e.start) {
                hi = mid - 1;
            } else if (cluster >= e.start + e.count) {
                lo = mid + 1;
            } else {
                return e;
            }
        }
        return null;
    }

    private void readFile(File f, long pos, byte[] dst, int off, int len) {
        try {
            RandomAccessFile raf = open.get(f);
            if (raf == null) {
                raf = new RandomAccessFile(f, "r");
                open.put(f, raf);
            }
            raf.seek(pos);
            int done = 0;
            while (done < len) {
                int r = raf.read(dst, off + done, len - done);
                if (r < 0) {
                    break;
                }
                done += r;
            }
        } catch (IOException ex) {
            // fichero desaparecido: se devuelve relleno de ceros
        }
    }

    public synchronized void writeSector(long lba, byte[] src, int off) {
        if (lba < 0 || lba >= diskSectors) {
            return;
        }
        overlay.put(lba, Arrays.copyOfRange(src, off, off + SECTOR));
        dirty = true;
        lastWriteMs = System.currentTimeMillis();
    }

    public boolean hasPendingWrites() {
        return dirty;
    }

    public long millisSinceLastWrite() {
        return System.currentTimeMillis() - lastWriteMs;
    }

    // ------------------------------------------------------ exportación al host
    /**
     * Exporta a la carpeta real los ficheros nuevos o modificados por el guest.
     * Nunca borra ni renombra nada de la carpeta.
     */
    public synchronized int flushToHost() {
        if (!dirty) {
            return 0;
        }
        int written = 0;
        try {
            byte[] bpb = new byte[SECTOR];
            readSector(PART_START, bpb, 0);
            int bps = u16(bpb, 11);
            int sp = bpb[13] & 0xFF;
            int rsv = u16(bpb, 14);
            int nf = bpb[16] & 0xFF;
            int fsz = u32(bpb, 36);
            int rootCl = u32(bpb, 44);
            if (bps != SECTOR || sp == 0 || nf == 0 || fsz <= 0) {
                return 0;
            }
            Set<Integer> touched = touchedClusters(rsv + nf * fsz, sp);
            written = exportDir(rootCl, folder, "", rsv, fsz, sp, rsv + nf * fsz, touched, 0);
            dirty = false;
        } catch (IOException | RuntimeException e) {
            System.out.println("[USB] Error exportando a la carpeta: " + e);
        }
        if (written > 0) {
            System.out.println("[USB] Exportados " + written + " fichero(s) nuevos/modificados a " + folder);
        }
        return written;
    }

    private Set<Integer> touchedClusters(int dataStartRel, int sp) {
        Set<Integer> t = new HashSet<>();
        for (Long lba : overlay.keySet()) {
            long rel = lba - PART_START;
            if (rel >= dataStartRel) {
                t.add((int) ((rel - dataStartRel) / sp) + 2);
            }
        }
        return t;
    }

    private int fatEntry(int cluster, int rsv) {
        byte[] s = new byte[SECTOR];
        readSector(PART_START + rsv + (cluster * 4L) / SECTOR, s, 0);
        return u32(s, (int) ((cluster * 4L) % SECTOR)) & 0x0FFFFFFF;
    }

    private byte[] readChain(int first, int rsv, int sp, int dataStartRel, long limit) {
        java.io.ByteArrayOutputStream bo = new java.io.ByteArrayOutputStream();
        byte[] sec = new byte[SECTOR];
        int c = first;
        int guard = 0;
        while (c >= 2 && c < 0x0FFFFFF8 && guard++ < 0x1000000 && bo.size() < limit) {
            for (int i = 0; i < sp && bo.size() < limit; i++) {
                readSector(PART_START + dataStartRel + (long) (c - 2) * sp + i, sec, 0);
                bo.write(sec, 0, SECTOR);
            }
            c = fatEntry(c, rsv);
        }
        return bo.toByteArray();
    }

    private int exportDir(int cluster, Path host, String path, int rsv, int fsz, int sp, int dataStartRel,
            Set<Integer> touched, int depth) throws IOException {
        if (depth > 16) {
            return 0;
        }
        byte[] dir = readChain(cluster, rsv, sp, dataStartRel, 64L << 20);
        int written = 0;
        StringBuilder lfn = new StringBuilder();
        for (int p = 0; p + 32 <= dir.length; p += 32) {
            int first = dir[p] & 0xFF;
            if (first == 0) {
                break;
            }
            int attr = dir[p + 11] & 0xFF;
            if (first == 0xE5) {
                lfn.setLength(0);
                continue;
            }
            if (attr == 0x0F) {
                StringBuilder part = new StringBuilder();
                int[] offs = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
                for (int o : offs) {
                    char ch = (char) (u16(dir, p + o));
                    if (ch == 0 || ch == 0xFFFF) {
                        break;
                    }
                    part.append(ch);
                }
                lfn.insert(0, part);
                continue;
            }
            if ((attr & 0x08) != 0) {
                lfn.setLength(0);
                continue;
            }
            String sn = new String(dir, p, 8, StandardCharsets.ISO_8859_1).trim();
            String se = new String(dir, p + 8, 3, StandardCharsets.ISO_8859_1).trim();
            String name = lfn.length() > 0 ? lfn.toString() : (se.isEmpty() ? sn : sn + "." + se);
            lfn.setLength(0);
            if (name.equals(".") || name.equals("..") || sn.startsWith(".")) {
                continue;
            }
            if (name.contains("/") || name.contains("\\") || name.contains(":")) {
                continue;
            }
            int start = (u16(dir, p + 20) << 16) | u16(dir, p + 26);
            long size = u32(dir, p + 28) & 0xFFFFFFFFL;
            String rel = path + "/" + name;
            Path target = host.resolve(name);
            if ((attr & 0x10) != 0) {
                if (!Files.isDirectory(target)) {
                    Files.createDirectories(target);
                }
                if (start >= 2) {
                    written += exportDir(start, target, rel, rsv, fsz, sp, dataStartRel, touched, depth + 1);
                }
            } else {
                long[] orig = original.get(rel);
                boolean changed = orig == null || orig[0] != start || orig[1] != size;
                if (!changed && start >= 2) {
                    long n = (size + (long) sp * SECTOR - 1) / ((long) sp * SECTOR);
                    for (int c = start; c < start + n && !changed; c++) {
                        changed = touched.contains(c);
                    }
                }
                if (changed) {
                    byte[] data = start >= 2 ? readChain(start, rsv, sp, dataStartRel, size) : new byte[0];
                    if (data.length > size) {
                        data = Arrays.copyOf(data, (int) size);
                    }
                    Files.write(target, data);
                    written++;
                }
            }
        }
        return written;
    }

    // ----------------------------------------------------------------- helpers
    private static void le16(byte[] b, int o, int v) {
        b[o] = (byte) v;
        b[o + 1] = (byte) (v >> 8);
    }

    private static void le32(byte[] b, int o, int v) {
        b[o] = (byte) v;
        b[o + 1] = (byte) (v >> 8);
        b[o + 2] = (byte) (v >> 16);
        b[o + 3] = (byte) (v >> 24);
    }

    private static int u16(byte[] b, int o) {
        return (b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8);
    }

    private static int u32(byte[] b, int o) {
        return (b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8) | ((b[o + 2] & 0xFF) << 16) | ((b[o + 3] & 0xFF) << 24);
    }
}
