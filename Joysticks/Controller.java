/*
 * The MIT License
 *
 * Copyright 2026 sistemas.
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
package pcp.Joysticks;

import pcp.gpu.Device;

/**
 *
 * @author sistemas
 */
public class Controller implements Device{

    private int tipo; // teclado, joystick, etc
    private int id; // mando 1, 2, 3 o 4
    private final int[] mapaTeclas;
    private boolean conectado;

    public Controller() {
        mapaTeclas = new int[22];
    }

    public void mapearTeclas(int tecla, int pos) {
        if (pos >= 0 && pos < mapaTeclas.length) {
            mapaTeclas[pos] = tecla;
        }
    }

    public boolean esTeclado() {
        /*
        0 - joystick
        1 - teclado
         */
        return tipo != 0;
    }

    public boolean estaConectado() {
        return conectado;
    }

    public int getMando() {
        return id;
    }

    @Override
    public boolean containsAddress(long address) {
        throw new UnsupportedOperationException("Not supported yet."); // Generated from nbfs://nbhost/SystemFileSystem/Templates/Classes/Code/GeneratedMethodBody
    }

    @Override
    public byte read(long offs) {
        throw new UnsupportedOperationException("Not supported yet."); // Generated from nbfs://nbhost/SystemFileSystem/Templates/Classes/Code/GeneratedMethodBody
    }

    @Override
    public void write(long offs, byte v) {
        throw new UnsupportedOperationException("Not supported yet."); // Generated from nbfs://nbhost/SystemFileSystem/Templates/Classes/Code/GeneratedMethodBody
    }

}
