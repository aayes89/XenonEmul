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

import java.util.HashMap;
import java.util.Map;
import pcp.gpu.Device;

/**
 * Controlador Ethernet RTL8169 (NIC) de Xbox 360 en 0xEA006000. Responde al
 * reset de hardware y consultas PHY de libxenon/XeLL.
 *
 * @author Slam
 */
public class XenonNic implements Device {

    public static final long BASE = 0xEA006000L;
    public static final long SIZE = 0x1000L;

    private final Map<Long, Integer> regs = new HashMap<>();

    @Override
    public boolean containsAddress(long address) {
        return address >= 0 && address < SIZE;
    }

    @Override
    public synchronized int readWord(long offs) {
        long aligned = offs & ~3L;
        int valLE = readRegLE(aligned);
        return Integer.reverseBytes(valLE);
    }

    private int readRegLE(long aligned) {
        System.out.println("[XenonNIC] lectura: " + aligned);
        switch ((int) aligned) {
            case 0x34: {
                // Byte 0x37 es ChipCmd: bit 0x10 = Reset en curso (debe volver a 0)
                int v = regs.getOrDefault(0x34L, 0);
                return v & ~0x10000000;
            }
            case 0x60: {
                // PHYAR: bit 31 (0x80000000) indica completado en lectura PHY,
                // o 0 en escritura PHY. Al leerlo tras escribir, alternamos el bit 31
                // para que libxenon salga del bucle de espera de PHY inmediatamente.
                int v = regs.getOrDefault(0x60L, 0);
                int toggled = v ^ 0x80000000;
                regs.put(0x60L, toggled);
                return toggled;
            }
            case 0x6C:
                // PHYStatus: bit 1 (0x02) = LinkStatus (0 = cable desconectado)
                return 0x00000000;
            default:
                return regs.getOrDefault(aligned, 0);
        }
    }

    @Override
    public synchronized void writeWord(long offs, int vBE) {
        long aligned = offs & ~3L;
        int vLE = Integer.reverseBytes(vBE);
        if (aligned == 0x34L) {
            // Limpiar bit Reset (0x10 en byte +3 -> 0x10000000 en palabra LE)
            vLE &= ~0x10000000;
        }
        System.out.println("[XenonNIC] escritura: " + aligned + " en: " + vLE);
        regs.put(aligned, vLE);
    }

    @Override
    public synchronized byte read(long offs) {
        int wordBE = readWord(offs & ~3L);
        int shift = 24 - 8 * (int) (offs & 3L);
        return (byte) (wordBE >>> shift);
    }

    @Override
    public synchronized void write(long offs, byte v) {
        long aligned = offs & ~3L;
        int wordBE = readWord(aligned);
        int shift = 24 - 8 * (int) (offs & 3L);
        int mask = 0xFF << shift;
        int nextBE = (wordBE & ~mask) | ((v & 0xFF) << shift);
        writeWord(aligned, nextBE);
    }
}
