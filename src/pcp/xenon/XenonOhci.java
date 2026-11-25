package pcp.xenon;
/*
 * The MIT License
 *
 * Copyright 2025 Slam.
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

import java.util.ArrayList;
import java.util.List;
import pcp.gpu.Device;
import pcp.system.Memory;

/**
 * Controlador USB OHCI 1.0 (registros little-endian accedidos con lwbrx/stwbrx,
 * estructuras DMA HCCA/ED/TD también little-endian). Implementa el hub raíz y
 * el procesamiento de las listas de control y bulk contra un dispositivo
 * {@link UsbMassStorage} conectado al primer puerto.
 * 
 * @autor Slam
 */
public class XenonOhci implements Device {

    public static final int SIZE = 0x100;
    private static final int NDP = 2;

    // bits HcInterruptStatus
    private static final int INT_WDH = 1 << 1;
    private static final int INT_RHSC = 1 << 6;

    // bits HcControl
    private static final int CTRL_CLE = 1 << 4;
    private static final int CTRL_BLE = 1 << 5;

    // códigos de condición
    private static final int CC_OK = 0;
    private static final int CC_STALL = 4;
    private static final int CC_NO_RESPONSE = 5;

    private final String name;
    private final Memory mem;
    private final UsbMassStorage[] devices = new UsbMassStorage[NDP];
    private final int[] portStatus = new int[NDP];

    private int control = 0xC0;           // USBSUSPEND tras reset
    private int cmdStatus = 0;
    private int intStatus = 0;
    private int intEnable = 0;
    private int hcca = 0;
    private int periodCurED = 0;
    private int ctrlHeadED = 0, ctrlCurED = 0, bulkHeadED = 0, bulkCurED = 0;
    private int hcDoneHead = 0;
    private int fmInterval = 0x27782EDF;
    private int periodicStart = 0;
    private int lsThreshold = 0x628;
    private long frameEpochNs = System.nanoTime();
    private final List<Integer> pendingDone = new ArrayList<>();
    private boolean trace = Boolean.getBoolean("xenon.usb.trace");
    private int traceCount = 0;

    private final Thread worker;

    public XenonOhci(String name, Memory mem) {
        this.name = name;
        this.mem = mem;
        for (int i = 0; i < NDP; i++) {
            portStatus[i] = 1 << 8; // PPS
        }
        worker = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    return;
                }
                synchronized (XenonOhci.this) {
                    process();
                }
            }
        }, "ohci-" + name);
        worker.setDaemon(true);
        worker.start();
    }

    /** Conecta un dispositivo al puerto indicado (0 o 1). */
    public synchronized void attach(int port, UsbMassStorage dev) {
        devices[port] = dev;
        portStatus[port] |= 1 | (1 << 16); // CCS + CSC
        intStatus |= INT_RHSC;
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
        // los registros OHCI se acceden por palabras
    }

    private int frameNumber() {
        return (int) (((System.nanoTime() - frameEpochNs) / 1_000_000L) & 0xFFFF);
    }

    private int readReg(int reg) {
        switch (reg) {
            case 0x00:
                return 0x10;
            case 0x04:
                return control;
            case 0x08:
                return cmdStatus;
            case 0x0C:
                process();
                return intStatus;
            case 0x10:
            case 0x14:
                return intEnable;
            case 0x18:
                return hcca;
            case 0x1C:
                return periodCurED;
            case 0x20:
                return ctrlHeadED;
            case 0x24:
                return ctrlCurED;
            case 0x28:
                return bulkHeadED;
            case 0x2C:
                return bulkCurED;
            case 0x30:
                process();
                return hcDoneHead;
            case 0x34:
                return fmInterval;
            case 0x38: {
                int fi = fmInterval & 0x3FFF;
                long ns = (System.nanoTime() - frameEpochNs) % 1_000_000L;
                int rem = (int) (fi - (ns * fi) / 1_000_000L);
                int frt = (frameNumber() & 1) << 31;
                return rem | frt;
            }
            case 0x3C:
                return frameNumber();
            case 0x40:
                return periodicStart;
            case 0x44:
                return lsThreshold;
            case 0x48:
                return 0x02000200 | NDP; // POTPGT=2, NPS=1, NDP
            case 0x4C:
                return 0;
            case 0x50:
                return 0;
            default:
                if (reg >= 0x54 && reg < 0x54 + 4 * NDP) {
                    return portStatus[(reg - 0x54) / 4];
                }
                return 0;
        }
    }

    private void writeReg(int reg, int v) {
        switch (reg) {
            case 0x04:
                control = v & 0x7FF;
                break;
            case 0x08:
                if ((v & 1) != 0) {
                    hardReset();
                }
                cmdStatus |= v & 0x6; // CLF, BLF
                process();
                break;
            case 0x0C:
                intStatus &= ~v;
                if ((v & INT_WDH) != 0) {
                    flushDone();
                }
                break;
            case 0x10:
                intEnable |= v;
                break;
            case 0x14:
                intEnable &= ~v;
                break;
            case 0x18:
                hcca = v & ~0xFF;
                break;
            case 0x1C:
                periodCurED = v & ~0xF;
                break;
            case 0x20:
                ctrlHeadED = v & ~0xF;
                break;
            case 0x24:
                ctrlCurED = v & ~0xF;
                break;
            case 0x28:
                bulkHeadED = v & ~0xF;
                break;
            case 0x2C:
                bulkCurED = v & ~0xF;
                break;
            case 0x34:
                fmInterval = v;
                break;
            case 0x40:
                periodicStart = v & 0x3FFF;
                break;
            case 0x44:
                lsThreshold = v & 0xFFF;
                break;
            case 0x50:
                break;
            default:
                if (reg >= 0x54 && reg < 0x54 + 4 * NDP) {
                    writePort((reg - 0x54) / 4, v);
                }
                break;
        }
        if (reg == 0x04 || reg == 0x20 || reg == 0x28) {
            process();
        }
    }

    private void hardReset() {
        control = 0xC0;
        cmdStatus = 0;
        intStatus = 0;
        intEnable = 0;
        hcca = 0;
        periodCurED = ctrlHeadED = ctrlCurED = bulkHeadED = bulkCurED = 0;
        hcDoneHead = 0;
        fmInterval = 0x27782EDF;
        pendingDone.clear();
        frameEpochNs = System.nanoTime();
        for (int i = 0; i < NDP; i++) {
            if (devices[i] != null) {
                portStatus[i] |= 1 | (1 << 16);
                intStatus |= INT_RHSC;
            }
        }
    }

    private void writePort(int p, int w) {
        int st = portStatus[p];
        if ((w & (1 << 0)) != 0) { // ClearPortEnable
            st &= ~(1 << 1);
        }
        if ((w & (1 << 1)) != 0 && (st & 1) != 0) { // SetPortEnable
            st |= 1 << 1;
        }
        if ((w & (1 << 2)) != 0 && (st & 2) != 0) { // SetPortSuspend
            st |= 1 << 2;
        }
        if ((w & (1 << 3)) != 0) { // ClearSuspendStatus
            st &= ~(1 << 2);
        }
        if ((w & (1 << 4)) != 0 && (st & 1) != 0) { // SetPortReset (completa al instante)
            st |= (1 << 1) | (1 << 20);
            st &= ~(1 << 4);
            if (devices[p] != null) {
                devices[p].reset();
            }
        }
        if ((w & (1 << 8)) != 0) {
            st |= 1 << 8;
        }
        st &= ~(w & 0x1F0000); // limpiar bits de cambio escribiendo 1
        portStatus[p] = st;
        if ((st & 0x1F0000) != 0) {
            intStatus |= INT_RHSC;
        }
    }

    // ------------------------------------------------------- listas y DMA
    private boolean operational() {
        return ((control >> 6) & 3) == 2;
    }

    private void process() {
        if (!operational() || hcca == 0) {
            return;
        }
        boolean any = false;
        if ((control & CTRL_CLE) != 0 && ctrlHeadED != 0) {
            any |= processList(ctrlHeadED);
        }
        if ((control & CTRL_BLE) != 0 && bulkHeadED != 0) {
            any |= processList(bulkHeadED);
        }
        if (any) {
            cmdStatus &= ~0x6;
        }
        flushDone();
    }

    private boolean processList(int head) {
        boolean any = false;
        int ed = head;
        int guard = 0;
        while (ed != 0 && guard++ < 128) {
            int d0 = rd(ed);
            int tail = rd(ed + 4) & ~0xF;
            int headp = rd(ed + 8);
            int next = rd(ed + 12) & ~0xF;
            boolean skip = (d0 & (1 << 14)) != 0;
            boolean halted = (headp & 1) != 0;
            if (!skip && !halted && (headp & ~0xF) != tail) {
                any |= runEd(ed, d0, tail);
            }
            ed = next;
        }
        return any;
    }

    private boolean runEd(int ed, int d0, int tail) {
        boolean any = false;
        for (int n = 0; n < 256; n++) {
            int headp = rd(ed + 8);
            if ((headp & 1) != 0 || (headp & ~0xF) == tail) {
                break;
            }
            int td = headp & ~0xF;
            int carry = (headp >> 1) & 1;
            int t0 = rd(td);
            int cbp = rd(td + 4);
            int nextTd = rd(td + 8) & ~0xF;
            int be = rd(td + 12);
            int cc = CC_OK;
            int remainingCbp = 0;

            int fa = d0 & 0x7F;
            int ep = (d0 >> 7) & 0xF;
            int edDir = (d0 >> 11) & 3;
            int dp = (t0 >> 19) & 3;
            int pid;
            if (edDir == 1) {
                pid = 1;
            } else if (edDir == 2) {
                pid = 2;
            } else {
                pid = dp; // 0 SETUP, 1 OUT, 2 IN
            }

            int len = 0;
            if (cbp != 0) {
                if (((cbp ^ be) & ~0xFFF) == 0) {
                    len = be - cbp + 1;
                } else {
                    len = (0x1000 - (cbp & 0xFFF)) + (be & 0xFFF) + 1;
                }
                if (len < 0) {
                    len = 0;
                }
            }
            int firstPage = cbp == 0 ? 0 : Math.min(len, 0x1000 - (cbp & 0xFFF));

            UsbMassStorage dev = findDevice(fa);
            if (dev == null) {
                cc = CC_NO_RESPONSE;
            } else if (ep == 0) {
                if (pid == 0) {
                    byte[] s = new byte[8];
                    for (int i = 0; i < 8 && i < len; i++) {
                        s[i] = rb(bufAddr(cbp, be, firstPage, i));
                    }
                    cc = dev.setup(s) ? CC_OK : CC_STALL;
                    if (trace && traceCount++ < 400) {
                        System.out.printf("[OHCI %s] SETUP fa=%d %02X %02X v=%02X%02X i=%02X%02X l=%02X%02X -> cc=%d%n",
                                name, fa, s[0], s[1], s[3], s[2], s[5], s[4], s[7], s[6], cc);
                    }
                } else if (pid == 2) {
                    byte[] r = dev.controlIn(len);
                    for (int i = 0; i < r.length; i++) {
                        wb(bufAddr(cbp, be, firstPage, i), r[i]);
                    }
                    remainingCbp = r.length >= len ? 0 : advance(cbp, be, firstPage, r.length);
                    if (trace && traceCount++ < 400) {
                        System.out.printf("[OHCI %s] CTL IN len=%d got=%d cbp=%08X be=%08X t0=%08X ed=%08X%n", name, len, r.length, cbp, be, t0, d0);
                    }
                } // pid == 1 (OUT de control, etapa de estado): sin datos que procesar
            } else if (pid == 2) {
                byte[] r = dev.bulkIn(ep, len);
                if (r == null) {
                    cc = CC_STALL;
                } else {
                    for (int i = 0; i < r.length; i++) {
                        wb(bufAddr(cbp, be, firstPage, i), r[i]);
                    }
                    remainingCbp = r.length >= len ? 0 : advance(cbp, be, firstPage, r.length);
                    if (trace && traceCount++ < 400) {
                        System.out.printf("[OHCI %s] BULK IN ep=%d len=%d got=%d%n", name, ep, len, r.length);
                    }
                }
            } else if (pid == 1) {
                byte[] data = new byte[len];
                for (int i = 0; i < len; i++) {
                    data[i] = rb(bufAddr(cbp, be, firstPage, i));
                }
                cc = dev.bulkOut(ep, data) ? CC_OK : CC_STALL;
                if (trace && traceCount++ < 400) {
                    System.out.printf("[OHCI %s] BULK OUT ep=%d len=%d cc=%d%n", name, ep, len, cc);
                }
            }

            // actualizar el TD y encolarlo como completado
            t0 = (t0 & 0x0FFFFFFF) | (cc << 28);
            wr(td, t0);
            wr(td + 4, cc == CC_OK ? remainingCbp : cbp);
            pendingDone.add(td);
            int newHead = nextTd | ((carry ^ 1) << 1);
            if (cc != CC_OK) {
                newHead |= 1; // ED detenido
            }
            wr(ed + 8, newHead);
            any = true;
        }
        return any;
    }

    private static int bufAddr(int cbp, int be, int firstPage, int i) {
        return i < firstPage ? cbp + i : (be & ~0xFFF) + (i - firstPage);
    }

    private static int advance(int cbp, int be, int firstPage, int n) {
        return bufAddr(cbp, be, firstPage, n);
    }

    private UsbMassStorage findDevice(int fa) {
        for (int p = 0; p < NDP; p++) {
            UsbMassStorage d = devices[p];
            if (d != null && (portStatus[p] & 2) != 0 && d.matches(fa)) {
                return d;
            }
        }
        return null;
    }

    private void flushDone() {
        if (pendingDone.isEmpty() || (intStatus & INT_WDH) != 0 || hcca == 0) {
            return;
        }
        for (int td : pendingDone) {
            wr(td + 8, hcDoneHead);
            hcDoneHead = td;
        }
        pendingDone.clear();
        int head = hcDoneHead;
        if ((intStatus & intEnable & ~INT_WDH) != 0) {
            head |= 1;
        }
        wr(hcca + 0x84, head);
        hcDoneHead = 0;
        intStatus |= INT_WDH;
    }
}
