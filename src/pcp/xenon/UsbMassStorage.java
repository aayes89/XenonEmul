package pcp.xenon;
/*
 * The MIT License
 *
 * Copyright 2026 Slam.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/**
 * Dispositivo USB de almacenamiento masivo (Bulk-Only Transport, SCSI
 * transparente) que expone un {@link FatFolderDisk}. Full-speed, un único
 * interfaz con endpoints bulk 0x81 (IN) y 0x02 (OUT).
 * 
 * @autor Slam
 */
public class UsbMassStorage {

    private static final int STALL = -1;

    private final FatFolderDisk disk;

    private int address = 0;
    private int pendingAddress = -1;
    private int config = 0;

    // etapa de control
    private byte[] ctlIn = new byte[0];
    private int ctlPos = 0;

    // transporte bulk-only
    private enum State { IDLE, DATA_IN, DATA_OUT, CSW }

    private State state = State.IDLE;
    private int cswTag;
    private int cswResidue;
    private int cswStatus;
    private int expectedLen;
    private byte[] inBuf = new byte[0];
    private int inPos = 0;
    private boolean streaming = false;
    private long streamOff;
    private long streamBytes;
    private boolean writing = false;
    private long writeLba;
    private long writeBytes;
    private final ByteArrayOutputStream writeAcc = new ByteArrayOutputStream();
    private int senseKey = 0, asc = 0, ascq = 0;

    public UsbMassStorage(FatFolderDisk disk) {
        this.disk = disk;
    }

    public synchronized void reset() {
        address = 0;
        pendingAddress = -1;
        config = 0;
        state = State.IDLE;
        streaming = false;
        writing = false;
        ctlIn = new byte[0];
        ctlPos = 0;
    }

    /** Resuelve si el dispositivo responde a esta dirección de función. */
    public synchronized boolean matches(int fa) {
        if (fa == address) {
            return true;
        }
        if (pendingAddress >= 0 && fa == pendingAddress) {
            address = pendingAddress;
            pendingAddress = -1;
            return true;
        }
        return false;
    }

    // ------------------------------------------------------------- descriptores
    private static byte[] deviceDescriptor() {
        return new byte[]{18, 1, 0x00, 0x02, 0, 0, 0, 64, (byte) 0x81, 0x07, 0x67, 0x55, 0x00, 0x01, 1, 2, 3, 1};
    }

    private static byte[] configDescriptor() {
        return new byte[]{
            9, 2, 32, 0, 1, 1, 0, (byte) 0x80, 50,
            9, 4, 0, 0, 2, 8, 6, 0x50, 0,
            7, 5, (byte) 0x81, 2, 0x00, 0x02, 0,
            7, 5, 0x02, 2, 0x00, 0x02, 0};
    }

    private static byte[] stringDescriptor(int idx) {
        if (idx == 0) {
            return new byte[]{4, 3, 0x09, 0x04};
        }
        String s;
        switch (idx) {
            case 1:
                s = "XenonEmul";
                break;
            case 2:
                s = "Virtual USB Disk";
                break;
            case 3:
                s = "0000000001";
                break;
            default:
                return null;
        }
        byte[] u = s.getBytes(StandardCharsets.UTF_16LE);
        byte[] r = new byte[2 + u.length];
        r[0] = (byte) r.length;
        r[1] = 3;
        System.arraycopy(u, 0, r, 2, u.length);
        return r;
    }

    // ---------------------------------------------------------------- control
    /** Procesa la etapa SETUP. Devuelve false si el dispositivo debe hacer STALL. */
    public synchronized boolean setup(byte[] s) {
        int bm = s[0] & 0xFF, br = s[1] & 0xFF;
        int wValue = (s[2] & 0xFF) | ((s[3] & 0xFF) << 8);
        int wLength = (s[6] & 0xFF) | ((s[7] & 0xFF) << 8);
        byte[] resp = new byte[0];
        ctlPos = 0;
        int type = (bm >> 5) & 3;
        if (type == 0) { // estándar
            switch (br) {
                case 0x00: // GET_STATUS
                    resp = new byte[]{0, 0};
                    break;
                case 0x01: // CLEAR_FEATURE
                case 0x03: // SET_FEATURE
                    break;
                case 0x05: // SET_ADDRESS
                    pendingAddress = wValue & 0x7F;
                    break;
                case 0x06: { // GET_DESCRIPTOR
                    int dt = wValue >> 8, di = wValue & 0xFF;
                    byte[] d = null;
                    if (dt == 1) {
                        d = deviceDescriptor();
                    } else if (dt == 2) {
                        d = configDescriptor();
                    } else if (dt == 3) {
                        d = stringDescriptor(di);
                    } else if (dt == 6) { // DEVICE_QUALIFIER (alta velocidad)
                        d = new byte[]{10, 6, 0x00, 0x02, 0, 0, 0, 64, 1, 0};
                    }
                    if (d == null) {
                        ctlIn = new byte[0];
                        return false;
                    }
                    resp = d;
                    break;
                }
                case 0x08:
                    resp = new byte[]{(byte) config};
                    break;
                case 0x09:
                    config = wValue & 0xFF;
                    break;
                case 0x0A:
                    resp = new byte[]{0};
                    break;
                case 0x0B:
                    break;
                default:
                    ctlIn = new byte[0];
                    return false;
            }
        } else if (type == 1) { // clase
            if (br == 0xFE) { // GET_MAX_LUN
                resp = new byte[]{0};
            } else if (br == 0xFF) { // Bulk-Only Mass Storage Reset
                state = State.IDLE;
                streaming = false;
                writing = false;
            } else {
                ctlIn = new byte[0];
                return false;
            }
        } else {
            ctlIn = new byte[0];
            return false;
        }
        if (resp.length > wLength) {
            resp = Arrays.copyOf(resp, wLength);
        }
        ctlIn = resp;
        return true;
    }

    /** Etapa de datos IN de control: devuelve hasta max bytes. */
    public synchronized byte[] controlIn(int max) {
        int n = Math.min(max, ctlIn.length - ctlPos);
        byte[] r = Arrays.copyOfRange(ctlIn, ctlPos, ctlPos + Math.max(0, n));
        ctlPos += Math.max(0, n);
        return r;
    }

    // ------------------------------------------------------------------- bulk
    /** Transferencia IN. Devuelve null para STALL. */
    public synchronized byte[] bulkIn(int ep, int max) {
        if (ep != 1) {
            return null;
        }
        if (state == State.DATA_IN) {
            byte[] out;
            if (streaming) {
                int n = (int) Math.min((long) max, streamBytes);
                out = new byte[n];
                readDisk(streamOff, out);
                streamOff += n;
                streamBytes -= n;
                if (streamBytes <= 0) {
                    streaming = false;
                    state = State.CSW;
                }
            } else {
                int n = Math.min(max, inBuf.length - inPos);
                out = Arrays.copyOfRange(inBuf, inPos, inPos + n);
                inPos += n;
                if (inPos >= inBuf.length) {
                    state = State.CSW;
                }
            }
            return out;
        }
        if (state == State.CSW) {
            byte[] c = new byte[13];
            c[0] = 'U';
            c[1] = 'S';
            c[2] = 'B';
            c[3] = 'S';
            le32(c, 4, cswTag);
            le32(c, 8, cswResidue);
            c[12] = (byte) cswStatus;
            state = State.IDLE;
            return c;
        }
        return null;
    }

    /** Transferencia OUT. Devuelve false para STALL. */
    public synchronized boolean bulkOut(int ep, byte[] data) {
        if (ep != 2) {
            return false;
        }
        if (state == State.DATA_OUT) {
            writeAcc.write(data, 0, data.length);
            while (writeAcc.size() >= FatFolderDisk.SECTOR && writeBytes > 0) {
                byte[] all = writeAcc.toByteArray();
                disk.writeSector(writeLba++, all, 0);
                writeBytes -= FatFolderDisk.SECTOR;
                writeAcc.reset();
                writeAcc.write(all, FatFolderDisk.SECTOR, all.length - FatFolderDisk.SECTOR);
            }
            if (writeBytes <= 0) {
                writing = false;
                writeAcc.reset();
                state = State.CSW;
            }
            return true;
        }
        if (data.length >= 31 && data[0] == 'U' && data[1] == 'S' && data[2] == 'B' && data[3] == 'C') {
            cbw(data);
            return true;
        }
        return state == State.CSW || state == State.IDLE;
    }

    private void cbw(byte[] c) {
        cswTag = u32(c, 4);
        expectedLen = u32(c, 8);
        boolean dirIn = (c[12] & 0x80) != 0;
        int op = c[15] & 0xFF;
        cswStatus = 0;
        inBuf = new byte[0];
        inPos = 0;
        streaming = false;
        writing = false;
        int dataLen = -1; // longitud real a devolver en DATA_IN
        switch (op) {
            case 0x00: // TEST UNIT READY
            case 0x1E: // PREVENT/ALLOW
            case 0x1B: // START STOP UNIT
            case 0x35: // SYNCHRONIZE CACHE
                clearSense();
                dataLen = 0;
                break;
            case 0x03: { // REQUEST SENSE
                byte[] s = new byte[18];
                s[0] = 0x70;
                s[2] = (byte) senseKey;
                s[7] = 10;
                s[12] = (byte) asc;
                s[13] = (byte) ascq;
                inBuf = Arrays.copyOf(s, Math.min(18, Math.max(0, c[19] & 0xFF)));
                clearSense();
                break;
            }
            case 0x12: { // INQUIRY
                byte[] q = new byte[36];
                q[1] = (byte) 0x80;
                q[2] = 0x04;
                q[3] = 0x02;
                q[4] = 31;
                put(q, 8, "XENON   ");
                put(q, 16, "VIRTUAL USB DISK");
                put(q, 32, "1.00");
                int alloc = ((c[18] & 0xFF) << 8) | (c[19] & 0xFF);
                inBuf = Arrays.copyOf(q, Math.min(36, alloc));
                clearSense();
                break;
            }
            case 0x1A: // MODE SENSE(6)
                inBuf = new byte[]{3, 0, 0, 0};
                clearSense();
                break;
            case 0x5A: // MODE SENSE(10)
                inBuf = new byte[]{0, 6, 0, 0, 0, 0, 0, 0};
                clearSense();
                break;
            case 0x25: { // READ CAPACITY(10)
                byte[] r = new byte[8];
                long last = disk.totalSectors() - 1;
                be32(r, 0, (int) last);
                be32(r, 4, FatFolderDisk.SECTOR);
                inBuf = r;
                clearSense();
                break;
            }
            case 0x28:
            case 0xA8: { // READ(10)/(12)
                long lba = u32be(c, 17) & 0xFFFFFFFFL;
                long cnt = op == 0x28 ? (((c[22] & 0xFF) << 8) | (c[23] & 0xFF))
                        : (u32be(c, 21) & 0xFFFFFFFFL);
                streaming = true;
                streamOff = lba * FatFolderDisk.SECTOR;
                streamBytes = cnt * FatFolderDisk.SECTOR;
                clearSense();
                if (streamBytes == 0) {
                    streaming = false;
                    dataLen = 0;
                }
                break;
            }
            case 0x2A:
            case 0xAA: { // WRITE(10)/(12)
                long lba = u32be(c, 17) & 0xFFFFFFFFL;
                long cnt = op == 0x2A ? (((c[22] & 0xFF) << 8) | (c[23] & 0xFF))
                        : (u32be(c, 21) & 0xFFFFFFFFL);
                writing = true;
                writeLba = lba;
                writeBytes = cnt * FatFolderDisk.SECTOR;
                writeAcc.reset();
                clearSense();
                if (writeBytes == 0) {
                    writing = false;
                    dataLen = 0;
                }
                break;
            }
            default:
                senseKey = 5;
                asc = 0x20;
                ascq = 0;
                cswStatus = 1;
                dataLen = 0;
                break;
        }
        if (writing) {
            cswResidue = 0;
            state = State.DATA_OUT;
            return;
        }
        if (streaming) {
            cswResidue = (int) Math.max(0, expectedLen - streamBytes);
            state = State.DATA_IN;
            return;
        }
        if (dataLen == 0) {
            inBuf = new byte[0];
        }
        cswResidue = Math.max(0, expectedLen - inBuf.length);
        if (inBuf.length == 0 && !(dirIn && expectedLen > 0)) {
            state = State.CSW;
        } else {
            state = State.DATA_IN;
        }
    }

    private void clearSense() {
        senseKey = 0;
        asc = 0;
        ascq = 0;
    }

    private void readDisk(long byteOff, byte[] out) {
        byte[] sec = new byte[FatFolderDisk.SECTOR];
        int done = 0;
        while (done < out.length) {
            long lba = (byteOff + done) / FatFolderDisk.SECTOR;
            int inSec = (int) ((byteOff + done) % FatFolderDisk.SECTOR);
            disk.readSector(lba, sec, 0);
            int n = Math.min(FatFolderDisk.SECTOR - inSec, out.length - done);
            System.arraycopy(sec, inSec, out, done, n);
            done += n;
        }
    }

    private static void put(byte[] b, int off, String s) {
        byte[] x = s.getBytes(StandardCharsets.US_ASCII);
        System.arraycopy(x, 0, b, off, x.length);
    }

    private static void le32(byte[] b, int o, int v) {
        b[o] = (byte) v;
        b[o + 1] = (byte) (v >> 8);
        b[o + 2] = (byte) (v >> 16);
        b[o + 3] = (byte) (v >> 24);
    }

    private static int u32(byte[] b, int o) {
        return (b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8) | ((b[o + 2] & 0xFF) << 16) | ((b[o + 3] & 0xFF) << 24);
    }

    private static void be32(byte[] b, int o, int v) {
        b[o] = (byte) (v >> 24);
        b[o + 1] = (byte) (v >> 16);
        b[o + 2] = (byte) (v >> 8);
        b[o + 3] = (byte) v;
    }

    private static int u32be(byte[] b, int o) {
        return ((b[o] & 0xFF) << 24) | ((b[o + 1] & 0xFF) << 16) | ((b[o + 2] & 0xFF) << 8) | (b[o + 3] & 0xFF);
    }
}
