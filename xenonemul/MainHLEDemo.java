package xenonemul;

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
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.Timer;

import pcp.gpu.FrameBuffer;
import pcp.gpu.FrameBufferPanel;
import pcp.xenon.TKernel;

/**
 * Main entry point correct for HLE XEX execution. Utiliza PPCSimulator para
 * mantener la integridad del bus, VRAM y CPU.
 *
 * @autor Slam
 *
 */
public class MainHLEDemo {

    public static void main(String[] args) {
        System.out.println("Starting HLE for Xbox360 by (Slam 2026) ...");
        if (args.length != 4) {
            System.err.println("This emulator needs:\n"
                    + "--nand=<path to nand.bin>\n"
                    + "--fuses=<path to fuses.txt>\n"
                    + "--onebl=<path to 1bl.bin>\n"
                    + "--elf=<path to stage2.elf>");
            System.exit(2);
        }
        String nandArg = "./src/pcp/TESTS/nand.bin";
        String fusesArg = "./src/pcp/fuses.txt";
        String oneblArg = "./src/pcp/TESTS/1bl.bin";
        String elfArg = "./src/pcp/TESTS/stage2.elf";
        
        for (String a : args) {
            if (a.startsWith("--nand=")) {
                nandArg = a.substring(7);
            } else if (a.startsWith("--fuses=")) {
                fusesArg = a.substring(8);
            } else if (a.startsWith("--onebl=")) {
                oneblArg = a.substring(8);
            } else if (a.startsWith("--elf=")) {
                elfArg = a.substring(6);
            }
        }

        // Configuramos la resolución típica del dashboard/xell
        int fbW = 1024;
        int fbH = 768;

        // Inicializar el hardware base mediante el PPCSimulator existente
        PPCSimulator simulator = new PPCSimulator(fbW, fbH);        
        /*simulator.load1BL("./src/pcp/TESTS/1bl.bin");*/
        try {
            simulator.loadFuses(Paths.get(fusesArg));
            simulator.map1BLRomOnly(oneblArg);
            simulator.loadNand(Paths.get(nandArg));
            // Carpeta USB: --usb=<carpeta>, -Dxenon.usb=<carpeta> o ./usb si existe
            String usb = System.getProperty("xenon.usb");
            for (String arg : args) {
                if (arg.startsWith("--usb=")) {
                    usb = arg.substring(6);
                }
            }
            if (usb == null && java.nio.file.Files.isDirectory(Paths.get("usb"))) {
                usb = "usb";
            }
            if (usb != null) {
                simulator.setUsbFolder(Paths.get(usb));
            }
        } catch (IOException e) {
            System.out.println(e.getMessage());
        }

        // Configurar el framebuffer al estilo Xenos (0x9E000000)
        simulator.setFbBase(PPCSimulator.FB_BASE_ELF);
        simulator.initializeXenonFramebuffer();
        //simulator.getFrameBuffer().setTileMode(2); // Desactivar Tiled Mode (modo lineal para el demo)

        // Inicializar el Kernel HLE (TKernel se conecta al simulador para atrapar llamadas 'sc')
        TKernel kernel = new TKernel(simulator);

        // Preparar la Interfaz de Usuario Gráfica (Swing)
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("XEX Emulator HLE - PPCSimulator by (Slam 2026)");
            FrameBuffer fb = simulator.getFrameBuffer();
            FrameBufferPanel panel = new FrameBufferPanel(fb.getWidth(), fb.getHeight(), fb.getFramebuffer());
            fb.attachPanel(panel);

            // Escalar si la pantalla es muy grande
            double sc = fb.getWidth() > 960 ? 960.0 / fb.getWidth() : 1.0;
            panel.setPreferredSize(new java.awt.Dimension((int) (fb.getWidth() * sc), (int) (fb.getHeight() * sc)));

            frame.add(panel, java.awt.BorderLayout.CENTER);
            frame.pack();
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // Bucle de repintado a ~30FPS para el Swing (desacoplado de la CPU)
            new Timer(33, e -> {
                if (fb.isDirty()) {
                    fb.present();
                }
                panel.repaint();
            }).start();
        });

        // Carga y Preparación del Ejecutable ELF (XeLL / LibXenon)
        File targetElf = new File(elfArg); // Asegúrate de la ruta correcta
        if (targetElf.exists()) {
            System.out.println("Cargando ELF Bare Metal: " + targetElf.getAbsolutePath());
            try {

                // El simulador ya tiene lógica especializada para ELF
                simulator.loadELF(targetElf.getAbsolutePath());

                // --- CONFIGURACIÓN CRÍTICA PARA LIBXENON / XELL ---
                // Activar el modo de 64 bits (SF = 1)
                simulator.getCpu().setMsr(simulator.getCpu().getMsr() | 0x8000000000000000L);

                // Configurar el Registro HRMOR para que el espejo de RAM (0x80000000) funcione correctamente.
                // XeLL usa el bit más significativo (0x8000000000000000) para saltos en modo real.
                simulator.getCpu().setHrmorValue(0x8000000000000000L);
                simulator.getCpu().setHid1Value(0x8000000000000000L);

                // Establecer la pila de hardware en la parte alta de la memoria
                long memorySize = simulator.getMemory().getMemorySize();
                long physicalTop = (memorySize - 16L) & ~15L;
                long stackTop = 0x8000000000000000L | physicalTop; // Usando el offset de 64 bits
                simulator.getCpu().setGPR(1, stackTop);

                System.out.println("ELF Cargado. PC inicial del Hilo: 0x" + Long.toHexString(simulator.getCpu().getPc()));

            } catch (Exception ex) {
                System.err.println("Error procesando el ELF: " + ex.getMessage());
                ex.printStackTrace();
                return;
            }
        } else {
            System.out.println("Archivo ELF no encontrado. Mostrando Framebuffer vacío.");
            return;
        }

        // Arrancar el loop de ejecución de la CPU
        System.out.println("Iniciando CPU (Modo Bare Metal 64-bit)...");
        simulator.getBus().setTraceUnmapped(true);
        simulator.start();

    }
}
