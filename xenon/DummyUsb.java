/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package pcp.xenon;

/**
 *
 * @author sistemas
 *
 */
import pcp.gpu.Device;

public class DummyUsb implements Device {

    private int ohciControl = 0;

    @Override
    public boolean containsAddress(long address) {
        return address >= 0 && address < 0x5000;
    }

    @Override
    public int readWord(long offs) {
        // offs es relativo a 0xEA002000:
        // +0x1000 = 0xEA003000 (EHCI 0)
        // +0x2000 = 0xEA004000 (OHCI 0)
        // +0x3000 = 0xEA005000 (EHCI 1)
        // +0x4000 = 0xEA006000 (OHCI 1)
        int block = (int) ((offs + 0x1000) & 0xF000);
        int reg = (int) ((offs + 0x1000) & 0x0FFF);
        int val = 0;

        if (block == 0x1000 || block == 0x3000) { // EHCI
            if (reg == 0x00) {
                val = 0x20;       // CAPLENGTH = 0x20
            } else if (reg == 0x04) {
                val = 0x00;  // HCSPARAMS: 0 puertos
            } else if (reg == 0x24) {
                val = 0x1000;// USBSTS: HCHalted = 1
            }
        } else if (block == 0x2000 || block == 0x4000) { // OHCI
            if (reg == 0x00) {
                val = 0x10;       // HcRevision = 1.0
            } else if (reg == 0x04) {
                val = ohciControl;
            } else if (reg == 0x08) {
                val = 0x00;  // HcCommandStatus: Reset listo
            } else if (reg == 0x48) {
                val = 0x00;  // HcRhDescriptorA: 0 puertos
            }
        }
        return Integer.reverseBytes(val); // libxenon usa lwbrx/stwbrx
    }

    @Override
    public void writeWord(long offs, int vBE) {
        int block = (int) (offs & 0xF000);
        int reg = (int) (offs & 0x0FFF);
        if ((block == 0x2000 || block == 0x4000) && reg == 0x04) {
            ohciControl = Integer.reverseBytes(vBE);
        }
    }

    @Override
    public byte read(long offs) {
        return (byte) (readWord(offs & ~3L) >>> (24 - 8 * (int) (offs & 3)));
    }

    @Override
    public void write(long offs, byte v) {
    }
}
