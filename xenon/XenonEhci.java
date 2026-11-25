package pcp.xenon;

import pcp.gpu.Device;
import pcp.system.Memory;

/**
 * Controlador USB 2.0 EHCI. Registros y estructuras DMA (QH/qTD) en
 * little-endian, accedidos por el guest con lwbrx/stwbrx. Implementa los
 * puertos del hub raíz (con enrutado a companion mediante CONFIGFLAG) y el
 * procesamiento de la lista asíncrona (control y bulk) contra un
 * {@link UsbMassStorage} de alta velocidad.
 */
public class XenonEhci implements Device {

    public static final int SIZE = 0x100;
    private static final int CAPLENGTH = 0x20;
    private static final int NDP = 2;

    private static final int CMD_RS = 1;
    private static final int CMD_HCRESET = 1 << 1;
    private static final int CMD_PSE = 1 << 4;
    private static final int CMD_ASE = 1 << 5;
    private static final int CMD_IAAD = 1 << 6;

    private static final int STS_USBINT = 1;
    private static final int STS_USBERRINT = 1 << 1;
    private static final int STS_PCD = 1 << 2;
    private static final int STS_IAA = 1 << 5;

    private static final int PORT_CSC = 1 << 1;
    private static final int PORT_PE = 1 << 2;
    private static final int PORT_PEC = 1 << 3;
    private static final int PORT_OCC = 1 << 5;
    private static final int PORT_PR = 1 << 8;
    private static final int PORT_PP = 1 << 12;
    private static final int PORT_PO = 1 << 13;

    private final String name;
    private final Memory mem;
    private final UsbMassStorage[] devices = new UsbMassStorage[NDP];
    private final int[] portsc = new int[NDP];

    private int usbcmd = 0x00080000;
    private int usbsts = 0;
    private int usbintr = 0;
    private int ctrlDsSegment = 0;
    private int periodicBase = 0;
    private int asyncAddr = 0;
    private int configFlag = 0;
    private final long epochNs = System.nanoTime();
    private final boolean trace = Boolean.getBoolean("xenon.usb.trace");
    private int traceCount = 0;

    public XenonEhci(String name, Memory mem) {
        this.name = name;
        this.mem = mem;
        for (int i = 0; i < NDP; i++) {
            portsc[i] = PORT_PO; // enrutado al companion hasta CONFIGFLAG=1
        }
        Thread worker = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    return;
                }
                synchronized (XenonEhci.this) {
                    processAsync();
                }
            }
        }, "ehci-" + name);
        worker.setDaemon(true);
        worker.start();
    }

    /** Conecta un dispositivo (alta velocidad) al puerto indicado. */
    public synchronized void attach(int port, UsbMassStorage dev) {
        devices[port] = dev;
        if ((portsc[port] & PORT_PO) == 0) {
            portsc[port] |= PORT_CSC;
            usbsts |= STS_PCD;
        }
    }

    // ------------------------------------------------------------------ DMA
    private int rd(int addr) {
        return Integer.reverseBytes(mem.readMemoryWord(addr & 0x1FFFFFFF));
    }

    private void wr(int addr, int v) {
        mem.writeMemoryWord(addr & 0x1FFFFFFF, Integer.reverseBytes(v));
    }

    private byte rb(int addr) {
        return mem.read(addr & 0x1FFFFFFF);
    }

    private void wb(int addr, byte v) {
        mem.write(addr & 0x1FFFFFFF, v);
    }

    // -------------------------------------------------------------- Device API
    @Override
    public boolean containsAddress(long address) {
        return address >= 0 && address < SIZE;
    }

    @Override
    public synchronized int readWord(long offs) {
        return Integer.reverseBytes(readReg((int) (offs & 0xFC)));
    }

    @Override
    public synchronized void writeWord(long offs, int vBE) {
        writeReg((int) (offs & 0xFC), Integer.reverseBytes(vBE));
    }

    @Override
    public byte read(long offs) {
        return (byte) (readWord(offs & ~3L) >>> (24 - 8 * (int) (offs & 3)));
    }

    @Override
    public void write(long offs, byte v) {
        // los registros EHCI se acceden por palabras
    }

    private int portValue(int p) {
        int v = portsc[p] | PORT_PP;
        boolean owned = (portsc[p] & PORT_PO) == 0;
        if (owned && devices[p] != null) {
            v |= 1; // CCS
        }
        return v;
    }

    private int readReg(int reg) {
        switch (reg) {
            case 0x00:
                return (0x0100 << 16) | CAPLENGTH;
            case 0x04:
                return (1 << 12) | (2 << 8) | NDP; // N_CC=1, N_PCC=2, N_PORTS
            case 0x08:
                return 0;
            case 0x20 + 0x00:
                return usbcmd;
            case 0x20 + 0x04: {
                processAsync();
                int s = usbsts & 0x3F;
                if ((usbcmd & CMD_RS) == 0) {
                    s |= 1 << 12; // HCHalted
                }
                if ((usbcmd & CMD_PSE) != 0) {
                    s |= 1 << 14;
                }
                if ((usbcmd & CMD_ASE) != 0) {
                    s |= 1 << 15;
                }
                return s;
            }
            case 0x20 + 0x08:
                return usbintr;
            case 0x20 + 0x0C:
                return (int) (((System.nanoTime() - epochNs) / 125_000L) & 0x3FFF);
            case 0x20 + 0x10:
                return ctrlDsSegment;
            case 0x20 + 0x14:
                return periodicBase;
            case 0x20 + 0x18:
                return asyncAddr;
            case 0x20 + 0x40:
                return configFlag;
            default:
                if (reg >= 0x20 + 0x44 && reg < 0x20 + 0x44 + 4 * NDP) {
                    return portValue((reg - (0x20 + 0x44)) / 4);
                }
                return 0;
        }
    }

    private void writeReg(int reg, int v) {
        switch (reg) {
            case 0x20 + 0x00:
                if ((v & CMD_HCRESET) != 0) {
                    hardReset();
                    break;
                }
                usbcmd = v & ~CMD_IAAD;
                if ((v & CMD_IAAD) != 0) {
                    processAsync();
                    usbsts |= STS_IAA;
                }
                break;
            case 0x20 + 0x04:
                usbsts &= ~(v & 0x3F);
                break;
            case 0x20 + 0x08:
                usbintr = v & 0x3F;
                break;
            case 0x20 + 0x10:
                ctrlDsSegment = v;
                break;
            case 0x20 + 0x14:
                periodicBase = v & ~0xFFF;
                break;
            case 0x20 + 0x18:
                asyncAddr = v & ~0x1F;
                break;
            case 0x20 + 0x40:
                setConfigFlag(v & 1);
                break;
            default:
                if (reg >= 0x20 + 0x44 && reg < 0x20 + 0x44 + 4 * NDP) {
                    writePort((reg - (0x20 + 0x44)) / 4, v);
                }
                break;
        }
        if (reg == 0x20 || reg == 0x38) {
            processAsync();
        }
    }

    private void hardReset() {
        usbcmd = 0x00080000;
        usbsts = 0;
        usbintr = 0;
        asyncAddr = 0;
        periodicBase = 0;
        setConfigFlag(0);
    }

    private void setConfigFlag(int f) {
        configFlag = f;
        for (int i = 0; i < NDP; i++) {
            if (f == 1) {
                if ((portsc[i] & PORT_PO) != 0) {
                    portsc[i] &= ~PORT_PO;
                    if (devices[i] != null) {
                        portsc[i] |= PORT_CSC;
                        usbsts |= STS_PCD;
                    }
                }
            } else {
                portsc[i] = (portsc[i] & ~PORT_PE) | PORT_PO;
            }
        }
    }

    private void writePort(int p, int w) {
        int st = portsc[p];
        st &= ~(w & (PORT_CSC | PORT_PEC | PORT_OCC)); // W1C
        if ((st & PORT_PO) == 0) {
            if ((w & PORT_PE) == 0) {
                st &= ~PORT_PE; // escribir 0 deshabilita
            }
            boolean wasReset = (st & PORT_PR) != 0;
            if ((w & PORT_PR) != 0) {
                st |= PORT_PR;
                st &= ~PORT_PE;
            } else if (wasReset) {
                st &= ~PORT_PR;
                if (devices[p] != null) {
                    st |= PORT_PE;
                    devices[p].reset();
                }
            }
        }
        if ((w & PORT_PO) != 0) {
            st |= PORT_PO;
            st &= ~PORT_PE;
        } else if ((portsc[p] & PORT_PO) != 0 && configFlag == 1) {
            st &= ~PORT_PO;
        }
        portsc[p] = st;
    }

    // -------------------------------------------------- lista asíncrona
    private UsbMassStorage findDevice(int fa) {
        for (int p = 0; p < NDP; p++) {
            UsbMassStorage d = devices[p];
            if (d != null && (portsc[p] & PORT_PE) != 0 && (portsc[p] & PORT_PO) == 0 && d.matches(fa)) {
                return d;
            }
        }
        return null;
    }

    private void processAsync() {
        if ((usbcmd & CMD_RS) == 0 || (usbcmd & CMD_ASE) == 0 || asyncAddr == 0) {
            return;
        }
        int start = asyncAddr;
        int qh = start;
        for (int guard = 0; guard < 64; guard++) {
            runQh(qh);
            int horiz = rd(qh);
            int next = horiz & ~0x1F;
            if ((horiz & 1) != 0 || next == 0 || next == start) {
                break;
            }
            qh = next;
        }
    }

    private void runQh(int qh) {
        int epchar = rd(qh + 4);
        int fa = epchar & 0x7F;
        int ep = (epchar >> 8) & 0xF;
        for (int n = 0; n < 128; n++) {
            int token = rd(qh + 24);
            if ((token & 0x80) == 0) {
                if ((token & 0x40) != 0) {
                    return; // detenido
                }
                int next = rd(qh + 16);
                if ((next & 1) != 0) {
                    return;
                }
                int qtd = next & ~0x1F;
                // cargar el qTD en el overlay de la QH
                wr(qh + 12, qtd);
                wr(qh + 16, rd(qtd));
                wr(qh + 20, rd(qtd + 4));
                wr(qh + 24, rd(qtd + 8));
                for (int i = 0; i < 5; i++) {
                    wr(qh + 28 + 4 * i, rd(qtd + 12 + 4 * i));
                }
                token = rd(qh + 24);
                if ((token & 0x80) == 0) {
                    continue; // qTD inactivo: se salta al siguiente
                }
            }
            executeActive(qh, fa, ep);
        }
    }

    private void executeActive(int qh, int fa, int ep) {
        int token = rd(qh + 24);
        int curQtd = rd(qh + 12);
        int pid = (token >> 8) & 3;
        int total = (token >> 16) & 0x7FFF;
        int toggle = (token >>> 31) & 1;
        int cpage = (token >> 12) & 7;
        int[] bufs = new int[5];
        for (int i = 0; i < 5; i++) {
            bufs[i] = rd(qh + 28 + 4 * i);
        }
        int off = bufs[0] & 0xFFF;
        boolean ok = true;
        boolean halted = false;
        int moved = 0;

        UsbMassStorage dev = findDevice(fa);
        if (dev == null) {
            halted = true;
            ok = false;
            token |= 1 << 3; // Transaction Error
        } else if (ep == 0) {
            if (pid == 2) { // SETUP
                byte[] s = new byte[8];
                for (int i = 0; i < 8; i++) {
                    s[i] = rb(bufAddr(bufs, off, i));
                }
                ok = dev.setup(s);
                moved = 8;
                if (trace && traceCount++ < 400) {
                    System.out.printf("[EHCI %s] SETUP fa=%d %02X %02X v=%02X%02X i=%02X%02X l=%02X%02X ok=%b%n",
                            name, fa, s[0], s[1], s[3], s[2], s[5], s[4], s[7], s[6], ok);
                }
            } else if (pid == 1) { // IN
                byte[] r = dev.controlIn(total);
                for (int i = 0; i < r.length; i++) {
                    wb(bufAddr(bufs, off, i), r[i]);
                }
                moved = r.length;
                if (trace && traceCount++ < 400) {
                    System.out.printf("[EHCI %s] CTL IN want=%d got=%d%n", name, total, r.length);
                }
            } else { // OUT (estado)
                moved = total;
            }
        } else if (pid == 1) { // bulk IN
            byte[] r = dev.bulkIn(ep, total);
            if (r == null) {
                ok = false;
            } else {
                for (int i = 0; i < r.length; i++) {
                    wb(bufAddr(bufs, off, i), r[i]);
                }
                moved = r.length;
            }
            if (trace && traceCount++ < 400) {
                System.out.printf("[EHCI %s] BULK IN ep=%d want=%d got=%d%n", name, ep, total, r == null ? -1 : r.length);
            }
        } else if (pid == 0) { // bulk OUT
            byte[] data = new byte[total];
            for (int i = 0; i < total; i++) {
                data[i] = rb(bufAddr(bufs, off, i));
            }
            ok = dev.bulkOut(ep, data);
            moved = total;
            if (trace && traceCount++ < 400) {
                System.out.printf("[EHCI %s] BULK OUT ep=%d len=%d ok=%b%n", name, ep, total, ok);
            }
        }

        boolean shortPacket = ok && pid == 1 && moved < total;
        int remaining = Math.max(0, total - moved);
        int packets = Math.max(1, (moved + 511) / 512);
        toggle ^= (packets & 1);
        int ntoken = (token & ~(0x80 | 0x7FFF0000 | 0xFF)) | (remaining << 16) | (toggle << 31);
        if (!ok) {
            ntoken |= 0x40; // Halted (STALL)
            usbsts |= STS_USBERRINT;
        }
        ntoken |= token & 0x1E; // conservar bits de error ya marcados (si los hubiera)
        if (halted) {
            ntoken |= 0x40;
            usbsts |= STS_USBERRINT;
        }
        wr(qh + 24, ntoken);
        if (curQtd != 0) {
            wr((curQtd & ~0x1F) + 8, ntoken);
        }
        if (shortPacket) {
            int alt = rd(qh + 20);
            if ((alt & 1) == 0) {
                wr(qh + 16, alt);
            }
        }
        if ((token & (1 << 15)) != 0) { // IOC
            usbsts |= STS_USBINT;
        }
    }

    private static int bufAddr(int[] bufs, int off, int i) {
        int pos = off + i;
        int page = pos >> 12;
        return (bufs[Math.min(page, 4)] & ~0xFFF) + (pos & 0xFFF);
    }
}
