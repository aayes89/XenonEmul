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

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.atomic.AtomicBoolean;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;

/** 
 * Reproduce el audio XMA original de bootanim, previamente decodificado a PCM WAV.
 * 
 * @autor Slam
 */
public final class BootAnimationAudio implements AutoCloseable {
    private final Path pcmFile;
    private final AtomicBoolean started = new AtomicBoolean();
    private volatile Clip clip;

    public BootAnimationAudio(Path pcmFile) {
        if (pcmFile == null) throw new IllegalArgumentException("Ruta PCM nula");
        this.pcmFile = pcmFile;
    }

    public static BootAnimationAudio fromDefaultNand() {
        return new BootAnimationAudio(Paths.get("nandDump/extracted/resources/bootanim.xex/audio-pcm.wav"));
    }

    /** Se llama al observar la transición guest; nunca inicia dos veces el mismo clip. */
    public void startOnce() {
        if (!started.compareAndSet(false, true)) return;
        try {
            if (!Files.isRegularFile(pcmFile)) throw new IOException("No existe audio PCM real: " + pcmFile);
            try (AudioInputStream in = AudioSystem.getAudioInputStream(pcmFile.toFile())) {
                Clip opened = AudioSystem.getClip();
                opened.open(in);
                clip = opened;
                opened.start();
                System.out.printf("[BOOTANIM-AUDIO] PLAY real-XMA-decoded PCM: %s, %.3f s, %d Hz, %d canales%n",
                        pcmFile, opened.getMicrosecondLength() / 1_000_000.0,
                        opened.getFormat().getSampleRate(), opened.getFormat().getChannels());
            }
        } catch (IOException | UnsupportedAudioFileException | LineUnavailableException | IllegalArgumentException ex) {
            System.err.println("[BOOTANIM-AUDIO] No se pudo reproducir el audio real: " + ex.getMessage());
        }
    }

    @Override public void close() {
        Clip current = clip;
        if (current != null) {
            current.stop();
            current.close();
            clip = null;
        }
    }
}
