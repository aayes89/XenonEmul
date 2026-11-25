package pcp.gpu;
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

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import pcp.cpu.CPU;

/*
 * Renders a diagnostic view of PPC instructions actually fetched by the guest CPU. 
 *
 * @autor Slam
 *
 */
public final class ExecutionTraceSnapshot {
    private ExecutionTraceSnapshot() {}

    public static void save(File file, CPU cpu) throws IOException {
        long[][] trace = cpu.getExecutionTrace();
        int width = 900, rowHeight = 19, header = 92;
        int rows = Math.max(1, trace.length);
        BufferedImage image = new BufferedImage(width, header + rows * rowHeight + 18,
                BufferedImage.TYPE_INT_RGB);
        Graphics2D g = image.createGraphics();
        try {
            g.setColor(new Color(0x0B1118)); g.fillRect(0, 0, image.getWidth(), image.getHeight());
            g.setFont(new Font(Font.MONOSPACED, Font.BOLD, 19));
            g.setColor(new Color(0xE6EDF3));
            g.drawString("JPPC-XENON | GUEST EXECUTION TRACE", 22, 30);
            g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 14));
            g.setColor(new Color(0x83D6A0));
            g.drawString(String.format("PC  %08X     INSTRUCTIONS  %d     HALTED  %s",
                    cpu.getPc(), cpu.getInstructionCount(), cpu.isHalted()), 22, 56);
            g.setColor(new Color(0x8B9AAA));
            g.drawString("ADDRESS       FETCHED PPC WORD (BIG-ENDIAN)", 22, 81);
            int y = header;
            for (long[] item : trace) {
                g.setColor(new Color(0x14212B)); g.fillRect(14, y - 14, width - 28, rowHeight);
                g.setColor(new Color(0x83D6A0));
                g.drawString(String.format("%08X      %08X", item[0], item[1]), 22, y);
                y += rowHeight;
            }
            if (trace.length == 0) {
                g.setColor(Color.WHITE); g.drawString("No guest instruction was fetched.", 22, y);
            }
        } finally { g.dispose(); }
        ImageIO.write(image, "PNG", file);
    }
}
