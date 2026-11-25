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
package pcp.xenon;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import xenonemul.PPCSimulator;
import pcp.cpu.CPU;
import pcp.gpu.Device;
import pcp.system.Bus;
import pcp.system.Memory;

/**
 * Kernel Xenon mínimo para ejecutar XEX directamente desde NAND/FlashFS.
 * Resuelve imports/exports reales en memoria y mantiene ABI PPC32.
 * 
 * @autor Slam
 */
public final class TKernel {

    private static final int PPC_LI_R0_BASE = 0x38000000; // li r0, imm16
    private static final int PPC_SC = 0x44000002;
    private static final int STATUS_SUCCESS = 0;
    private static final int STATUS_NOT_IMPLEMENTED = 0xC0000002;
    private static final int STATUS_INVALID_HANDLE = 0xC0000008;
    private static final int STATUS_INVALID_PARAMETER = 0xC000000D;
    private static final int STATUS_OBJECT_NAME_NOT_FOUND = 0xC0000034;
    private static final int STATUS_TIMEOUT = 0x00000102;
    private static final int FILE_OPEN = 1;

    private final PPCSimulator simulator;
    private final Bus bus;
    private final Memory memory;
    private final Map<Integer, String> namedKernelOrdinals = new LinkedHashMap<>();
    private final Map<String, Long> importThunkAddresses = new HashMap<>();
    private final Map<String, Map<Integer, Long>> moduleExports = new HashMap<>();
    private final Map<String, XenonFlashModuleLoader.Module> loadedModules = new HashMap<>();

    private long bootAnimationImageBase;
    private long bootAnimationImageSize;
    private boolean bootAnimationModuleLoaded;
    private boolean bootAnimationModuleHasEntryPoint;
    private XenonFlashModuleLoader.Module bootAnimationModule;
    private int installedThunks;
    private int unknownThunks;
    private boolean enginesInitialized;
    private boolean ringBufferInitialized;
    private boolean displayModeSet;
    private volatile boolean bootAnimationRequested;
    private final Map<String, Long> bootAnimationResources = new LinkedHashMap<>();
    private final Map<Integer, Long> dispatchCounts = new ConcurrentHashMap<>();
    private XenonFlashFs flashFs;
    private final Map<Integer, GuestFile> guestFiles = new ConcurrentHashMap<>();
    private final Map<Integer, CPU> guestThreads = new ConcurrentHashMap<>();
    // Mapa handle → dirección del objeto de hilo ficticio en memoria guest
    private final Map<Integer, Long> guestThreadObjects = new ConcurrentHashMap<>();
    private int nextGuestHandle = 0x100;
    private long ringBufferBase;
    private long ringBufferSize;
    private boolean cpuCrashGuarded = false;
    private volatile long forcedBootAnimEntry = 0;
    private volatile long forcedBootAnimCtx = 0;
    private volatile boolean bootAnimThreadStarted = false;

    // Heap dinámico para NtAllocateVirtualMemory (0x84000000 .. 0x91800000, por debajo de dash.xex en 0x92000000)
    private long nextVirtualAlloc = 0x84000000L;
    // Pilas y bloques TLS de hilos (0x83000000 .. 0x83FF0000)
    private long nextThreadRegion = 0x83F00000L;
    // Tabla TLS global por índice (0..63)
    private final boolean[] tlsSlotsInUse = new boolean[64];
    private final Map<CPU, long[]> threadTlsValues = new ConcurrentHashMap<>();

    public TKernel(PPCSimulator simulator) {
        if (simulator == null) {
            throw new IllegalArgumentException("PPCSimulator no puede ser null");
        }
        this.simulator = simulator;
        this.bus = simulator.getBus();
        this.memory = simulator.getMemory();
        loadKnownNames();
        simulator.setSystemCallHandler((cpu, serviceNumber) -> {
            long count = dispatchCounts.merge(serviceNumber, 1L, Long::sum);
            long pc = cpu.getPc();
            long lr = cpu.getLr();
            long[] args = new long[8];
            for (int i = 0; i < args.length; i++) {
                args[i] = cpu.getGPRVal(i + 3) & 0xFFFFFFFFL;
            }

            if ((serviceNumber & 0x10000) != 0) {
                int xamOrd = serviceNumber & 0xFFFF;
                if (count <= 5) {
                    System.out.printf("[TKERNEL-TRAP] Unresolved XAM import ordinal=%d PC=%08X LR=%08X "
                            + "r3=%08X r4=%08X r5=%08X r6=%08X -> STATUS_NOT_IMPLEMENTED%n",
                            xamOrd, pc, lr, args[0], args[1], args[2], args[3]);
                }
                return unsigned(STATUS_NOT_IMPLEMENTED);
            }

            long result = invokeKernelExport(serviceNumber, cpu) & 0xFFFFFFFFL;
            String export = namedKernelOrdinals.getOrDefault(serviceNumber, "UNKNOWN");
            if (result == unsigned(STATUS_NOT_IMPLEMENTED) || count <= 3 || serviceNumber == 678) {
                System.out.printf("[TKERNEL] SC #%d ordinal=%d name=%s PC=%08X LR=%08X "
                        + "r3=%08X r4=%08X r5=%08X r6=%08X r7=%08X r8=%08X r9=%08X r10=%08X result=%08X%n",
                        count, serviceNumber, export, pc, lr,
                        args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], result);
            }
            return result;
        });

        // Guard PC=0
        Thread pcGuard = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    return;
                }
                for (CPU t : guestThreads.values()) {
                    if (t.getPc() == 0) {
                        System.out.println("[TKERNEL] Hilo saltó a PC=0, detenido");
                        t.halt();
                    }
                }
            }
        }, "pc-guard");
        pcGuard.setDaemon(true);
        pcGuard.start();

        // Present periódico: garantiza que el framebuffer se vuelque a pantalla
        Thread periodicPresent = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    return;
                }
                try {
                    simulator.getFrameBuffer().present();
                } catch (Exception ignored) {
                }
            }
        }, "periodic-present");
        periodicPresent.setDaemon(true);
        periodicPresent.start();

        // Forzar patrón + present tras 300 ms (por si Aurora nunca llega a VdSwap)
        /* Thread forcePresent = new Thread(() -> {
            try {
                Thread.sleep(300);
                System.out.println("[TKERNEL] Forzando patrón inicial en framebuffer");
                Device fb = simulator.getFrameBuffer();
                for (long y = 0; y < 600; y++) {
                    for (long x = 0; x < 800; x++) {
                        fb.writeWord(y * 3200 + x * 4, 0x00FF0000);
                    }
                }

                simulator.getFrameBuffer().present();
                System.out.println("[TKERNEL] present() ejecutado");

            } catch (Exception e) {
                System.out.println("[TKERNEL] forcePresent error: " + e.getMessage());
            }
        }, "force-present");
        forcePresent.setDaemon(true);
        forcePresent.start();*/
    }

    private void loadKnownNames() {
        namedKernelOrdinals.put(13, "ExCreateEvent");
        namedKernelOrdinals.put(16, "ExGetXConfigSetting");
        namedKernelOrdinals.put(17, "ExGetXConfigSetting");
        namedKernelOrdinals.put(18, "ExRegisterTitleTerminateNotification");
        namedKernelOrdinals.put(21, "ExCreateMutant");
        namedKernelOrdinals.put(24, "ExCreateThread");
        namedKernelOrdinals.put(25, "ExTerminateThread");
        namedKernelOrdinals.put(33, "IoCreateDevice_Stub");
        namedKernelOrdinals.put(40, "ExAllocatePool");
        namedKernelOrdinals.put(41, "ExAllocatePoolWithTag");
        namedKernelOrdinals.put(42, "ExAllocatePoolTypeWithTag");
        namedKernelOrdinals.put(43, "ExFreePool");
        namedKernelOrdinals.put(82, "KeInitializeEvent");
        namedKernelOrdinals.put(83, "KeSetEvent");
        namedKernelOrdinals.put(90, "KeDelayExecutionThread");
        namedKernelOrdinals.put(95, "KeGetCurrentThread");
        namedKernelOrdinals.put(102, "KeGetCurrentProcessType");
        namedKernelOrdinals.put(109, "KeInitializeSemaphore");
        namedKernelOrdinals.put(110, "KeInitializeSpinLock");
        namedKernelOrdinals.put(111, "KeSetBasePriorityThread");
        namedKernelOrdinals.put(116, "KeSetPriorityThread");
        namedKernelOrdinals.put(117, "KeSetAffinityThread");
        namedKernelOrdinals.put(125, "KeWaitForSingleObject");
        namedKernelOrdinals.put(131, "KePulseEvent");
        namedKernelOrdinals.put(132, "KeQuerySystemTime");
        namedKernelOrdinals.put(133, "KeQueryPerformanceCounter");
        namedKernelOrdinals.put(134, "KeQueryPerformanceFrequency");
        namedKernelOrdinals.put(136, "KeQuerySystemTime");
        namedKernelOrdinals.put(143, "KeRaiseIrqlToDpcLevel");
        namedKernelOrdinals.put(146, "KeResumeThread");
        namedKernelOrdinals.put(151, "KeSetTimer");
        namedKernelOrdinals.put(153, "KeSetTimerEx");
        namedKernelOrdinals.put(162, "KeDelayExecutionThread");
        namedKernelOrdinals.put(155, "KeTlsAlloc");
        namedKernelOrdinals.put(156, "KeTlsFree");
        namedKernelOrdinals.put(157, "KeTlsGetValue");
        namedKernelOrdinals.put(158, "KeTlsSetValue");
        namedKernelOrdinals.put(186, "ObReferenceObjectByHandle");
        namedKernelOrdinals.put(190, "VdSetSurface_Stub");
        namedKernelOrdinals.put(194, "MmAllocatePhysicalMemory");
        namedKernelOrdinals.put(195, "MmAllocatePhysicalMemoryEx");
        namedKernelOrdinals.put(198, "MmFreePhysicalMemory");
        namedKernelOrdinals.put(200, "MmGetPhysicalAddress");
        namedKernelOrdinals.put(202, "MmQueryAllocationSize");
        namedKernelOrdinals.put(204, "NtAllocateVirtualMemory");
        namedKernelOrdinals.put(205, "NtFreeVirtualMemory");
        namedKernelOrdinals.put(206, "NtProtectVirtualMemory");
        namedKernelOrdinals.put(207, "NtQueryVirtualMemory");
        namedKernelOrdinals.put(210, "NtCreateFile");
        namedKernelOrdinals.put(211, "NtCreateDirectoryObject_Stub");
        namedKernelOrdinals.put(213, "NtOpenDirectoryObject_Stub");
        namedKernelOrdinals.put(215, "NtOpenFile");
        namedKernelOrdinals.put(223, "NtQueryInformationProcess_Stub");
        namedKernelOrdinals.put(231, "NtOpenSymbolicLinkObject");
        namedKernelOrdinals.put(232, "NtQueryVolumeInformationFile_Stub");
        namedKernelOrdinals.put(240, "NtReadFile");
        namedKernelOrdinals.put(245, "NtQueryPerformanceCounter");
        namedKernelOrdinals.put(247, "NtQueryInformationFile");
        namedKernelOrdinals.put(248, "NtQueryInformationFile");
        namedKernelOrdinals.put(253, "NtClose");
        namedKernelOrdinals.put(255, "PsCreateSystemThreadEx_Stub");
        namedKernelOrdinals.put(261, "NtSetInformationThread");
        namedKernelOrdinals.put(272, "NtQueryInformationThread");
        namedKernelOrdinals.put(275, "NtAllocateVirtualMemory");
        namedKernelOrdinals.put(276, "NtFreeVirtualMemory");
        namedKernelOrdinals.put(283, "NtResumeThread");
        namedKernelOrdinals.put(293, "RtlLeaveCriticalSection");
        namedKernelOrdinals.put(298, "RtlInitAnsiString");
        namedKernelOrdinals.put(299, "RtlInitUnicodeString");
        namedKernelOrdinals.put(300, "RtlEnterCriticalSection");
        namedKernelOrdinals.put(301, "RtlLeaveCriticalSection");
        namedKernelOrdinals.put(302, "RtlInitializeCriticalSection");
        namedKernelOrdinals.put(303, "RtlInitializeCriticalSectionAndSpinCount");
        namedKernelOrdinals.put(304, "RtlTryEnterCriticalSection");
        namedKernelOrdinals.put(309, "RtlEnterCriticalSection");
        namedKernelOrdinals.put(310, "RtlLeaveCriticalSection");
        namedKernelOrdinals.put(311, "RtlInitializeCriticalSection");
        namedKernelOrdinals.put(320, "RtlRaiseStatus_Stub");
        namedKernelOrdinals.put(322, "RtlRaiseException");
        namedKernelOrdinals.put(323, "RtlUnwind_Stub");
        namedKernelOrdinals.put(338, "RtlTimeFieldsToTime");
        namedKernelOrdinals.put(340, "RtlTimeToTimeFields");
        namedKernelOrdinals.put(341, "RtlTryEnterCriticalSection");
        namedKernelOrdinals.put(394, "KeSetTimerEx_Alias");
        namedKernelOrdinals.put(404, "XamNotifyCreateListener_Stub");
        namedKernelOrdinals.put(405, "Unknown405_Stub");
        namedKernelOrdinals.put(446, "VdGetCurrentDisplayInformation");
        namedKernelOrdinals.put(438, "VdSetSurface_Stub");
        namedKernelOrdinals.put(447, "VdGetCurrentDisplayMode");
        namedKernelOrdinals.put(450, "VdInitializeEngines");
        namedKernelOrdinals.put(451, "VdInitializeRingBuffer");
        namedKernelOrdinals.put(467, "VdSetDisplayMode");
        namedKernelOrdinals.put(468, "VdSetDisplayModeOverride");
        namedKernelOrdinals.put(469, "VdSetDisplayMode_Stub");
        namedKernelOrdinals.put(473, "VdSwap");
        namedKernelOrdinals.put(476, "VdShutdownEngines");
        namedKernelOrdinals.put(477, "VdTurnDisplayOff");
        namedKernelOrdinals.put(479, "VdSetDisplayModeOverride_Stub");
        namedKernelOrdinals.put(548, "XMACreateContext");
        namedKernelOrdinals.put(550, "XMAReleaseContext");
        namedKernelOrdinals.put(650, "XexGetModuleSection");
        namedKernelOrdinals.put(655, "XexGetProcedureAddress");
        namedKernelOrdinals.put(678, "AniStartBootAnimation");
        namedKernelOrdinals.put(808, "XamBuildResourceLocator");
        namedKernelOrdinals.put(820, "XamEnumerate");
        namedKernelOrdinals.put(859, "XamGetSystemVersion");
        namedKernelOrdinals.put(902, "XamTaskSchedule");
    }

    private static final class GuestFile {

        final XenonFlashFs.Entry entry;
        long position;

        GuestFile(XenonFlashFs.Entry entry) {
            this.entry = entry;
        }
    }

    public void attachFlashFileSystem(XenonFlashFs fs) {
        this.flashFs = fs;
        System.out.println("[TKERNEL] FlashFS attached: "
                + (fs == null ? "none" : fs.rootInfo().toString()));
    }

    public void registerModuleMetadata(XenonFlashModuleLoader.Module module) throws IOException {
        byte[] peImage = module.getXex().image(Collections.emptyList());
        String key = module.getName().toLowerCase(Locale.ROOT);
        loadedModules.put(key, module);
        Map<Integer, Long> exports = module.getXex().readExports(peImage);
        moduleExports.put(key, exports);

        long base = module.getImage().getImageBase() & 0xFFFFFFFFL;
        int mappedRes = 0;
        for (XexFile.Resource res : module.getXex().resources()) {
            long rva = (res.address & 0xFFFFFFFFL) - base;
            if (rva >= 0 && rva + res.size <= peImage.length && res.size > 0) {
                byte[] resBytes = Arrays.copyOfRange(peImage, (int) rva, (int) (rva + res.size));
                memory.writeBlock(res.address, resBytes);
                if ("bootanim.xex".equals(key)) {
                    bootAnimationResources.put(res.name.toLowerCase(Locale.ROOT), res.address);
                }
                mappedRes++;
            }
        }
        if ("bootanim.xex".equals(key)) {
            this.bootAnimationModule = module;
            registerBootAnimationModule(module.getImage());
        }
        System.out.printf("[TKERNEL] METADATA %s: exports=%d resourcesMapped=%d/%d%n",
                module.getName(), exports.size(), mappedRes, module.getXex().resources().size());
    }

    public Long getModuleExport(String moduleName, int ordinal) {
        if (moduleName == null) {
            return null;
        }
        Map<Integer, Long> map = moduleExports.get(moduleName.toLowerCase(Locale.ROOT));
        return map == null ? null : map.get(ordinal);
    }

    private static boolean isExecutableRva(XenonPeLoader.LoadedImage img, long rva) {
        for (XenonPeLoader.Section sec : img.getSections()) {
            if ((sec.getCharacteristics() & 0x20000000L) != 0) {
                long start = sec.getVirtualAddress();
                long end = start + Math.max(sec.getVirtualSize(), sec.getRawSize());
                if (rva >= start && rva < end) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean isExecutableExport(String libraryName, long targetVa) {
        if (libraryName == null || targetVa == 0) {
            return false;
        }
        XenonFlashModuleLoader.Module targetMod = loadedModules.get(libraryName.toLowerCase(Locale.ROOT));
        if (targetMod == null) {
            return false;
        }
        long base = targetMod.getImage().getImageBase() & 0xFFFFFFFFL;
        if (targetVa < base) {
            return false;
        }
        return isExecutableRva(targetMod.getImage(), targetVa - base);
    }

    private int patchCodeThunk(String callerModule, String libraryName, long thunkAddr, int ordinal) {
        int ord16 = ordinal & 0xFFFF;
        if ("xboxkrnl.exe".equalsIgnoreCase(libraryName)) {
            importThunkAddresses.put(importKey(callerModule, "xboxkrnl.exe", ord16), thunkAddr);
            memory.writeMemoryWord(thunkAddr, PPC_LI_R0_BASE | ord16);
            memory.writeMemoryWord(thunkAddr + 4, PPC_SC);
            memory.writeMemoryWord(thunkAddr + 8, 0x4E800020);
            if (!namedKernelOrdinals.containsKey(ord16)) {
                unknownThunks++;
            }
            installedThunks++;
            return 1;
        }
        Long target = getModuleExport(libraryName, ord16);
        if (target != null && target != 0 && isExecutableExport(libraryName, target)) {
            int hi = (int) ((target >>> 16) & 0xFFFF);
            int lo = (int) (target & 0xFFFF);
            memory.writeMemoryWord(thunkAddr, 0x3D800000 | hi);
            memory.writeMemoryWord(thunkAddr + 4, 0x618C0000 | lo);
            memory.writeMemoryWord(thunkAddr + 8, 0x7D8903A6);
            memory.writeMemoryWord(thunkAddr + 12, 0x4E800420);
            return 2;
        }
        if ("xam.xex".equalsIgnoreCase(libraryName)
                && (ord16 < 400 || namedKernelOrdinals.containsKey(ord16))) {
            importThunkAddresses.put(importKey(callerModule, "xboxkrnl.exe", ord16), thunkAddr);
            memory.writeMemoryWord(thunkAddr, PPC_LI_R0_BASE | ord16);
            memory.writeMemoryWord(thunkAddr + 4, PPC_SC);
            memory.writeMemoryWord(thunkAddr + 8, 0x4E800020);
            if (!namedKernelOrdinals.containsKey(ord16)) {
                unknownThunks++;
            }
            installedThunks++;
            return 1;
        }
        memory.writeMemoryWord(thunkAddr, 0x38600000);
        memory.writeMemoryWord(thunkAddr + 4, 0x4E800020);
        return 3;
    }

    public void resolveAllImports(XenonFlashModuleLoader.Module module, byte[] peImage) throws IOException {
        int kernelCalls = 0;
        int moduleCalls = 0;
        int unresolvedCalls = 0;
        int dataLinks = 0;

        for (XexFile.ImportEntry entry : module.getXex().importEntries(peImage)) {
            long thunk = entry.thunkAddress & 0xFFFFFFFFL;
            if (thunk == 0 || (thunk & 3L) != 0) {
                continue;
            }

            boolean inCodeSection = isExecutableRva(module.getImage(), entry.thunkRva);
            if (entry.typeFlag == 1 || inCodeSection) {
                int ordinal = entry.ordinal;
                int nextWord = memory.readMemoryWord(thunk + 4);
                if (((nextWord >>> 24) & 0xFF) == 1) {
                    ordinal = nextWord & 0x00FFFFFF;
                }
                int kind = patchCodeThunk(module.getName(), entry.library, thunk, ordinal);
                if (kind == 1) {
                    kernelCalls++;
                } else if (kind == 2) {
                    moduleCalls++;
                } else {
                    unresolvedCalls++;
                }
            } else {
                Long target = getModuleExport(entry.library, entry.ordinal);
                if (target != null && target != 0) {
                    memory.writeMemoryWord(thunk, (int) (target & 0xFFFFFFFFL));
                } else {
                    long varAddr = (0x8FFF0000L + ((entry.ordinal & 0x3FFL) * 64L)) & 0xFFFFFFFFL;
                    memory.writeMemoryWord(varAddr, 0x02000000);
                    memory.writeMemoryWord(varAddr + 4, 0x00000001);
                    memory.writeMemoryWord(thunk, (int) varAddr);
                }
                dataLinks++;
            }
        }
        java.util.List<XexFile.ImportLibrary> libs = module.getImports();
        long base = module.getImage().getImageBase() & 0xFFFFFFFFL;
        int extraThunks = 0;
        for (XenonPeLoader.Section sec : module.getImage().getSections()) {
            if ((sec.getCharacteristics() & 0x20000000L) == 0) {
                continue;
            }
            long start = base + sec.getVirtualAddress();
            long end = start + Math.max(sec.getVirtualSize(), sec.getRawSize());
            for (long addr = start; addr + 16L <= end; addr += 4L) {
                int w0 = memory.readMemoryWord(addr);
                if ((w0 & 0xFF000000) == 0x01000000) {
                    int libIdx = (w0 >>> 16) & 0xFF;
                    int ord = w0 & 0xFFFF;
                    if (libIdx < libs.size() && ord > 0) {
                        int kind = patchCodeThunk(module.getName(), libs.get(libIdx).name, addr, ord);
                        if (kind == 1) {
                            kernelCalls++;
                        } else if (kind == 2) {
                            moduleCalls++;
                        } else {
                            unresolvedCalls++;
                        }
                        extraThunks++;
                        addr += 12L;
                    }
                }
            }
        }
        System.out.printf("[TKERNEL] %s imports: kernel=%d moduleExports=%d unresolvedTraps=%d data=%d extraTextThunks=%d%n",
                module.getName(), kernelCalls, moduleCalls, unresolvedCalls, dataLinks, extraThunks);
    }

    public synchronized void startRetailThread(CPU cpu, XenonFlashModuleLoader.Module module, long entryPoint, long r3Arg, long r4Arg, long threadExit) {
        if (entryPoint == 0) {
            throw new IllegalArgumentException("Entry point nulo para " + module.getName());
        }
        long regionBase = nextThreadRegion;
        nextThreadRegion -= 0x40000L;
        if (nextThreadRegion < 0x82000000L) {
            nextThreadRegion = 0x83F00000L;
        }

        long stackTop = (regionBase + 0x38000L) & ~15L;
        long tebBase = (regionBase + 0x39000L) & ~15L;
        long tlsBase = (regionBase + 0x3A000L) & ~15L;

        XexFile.TlsInfo tls = module.getXex().tlsInfo();
        if (tls != null && tls.dataSize > 0 && tls.dataSize <= 0x4000L) {
            for (long i = 0; i < tls.dataSize; i++) {
                byte b = (i < tls.rawDataSize && tls.rawDataAddress != 0)
                        ? memory.read(tls.rawDataAddress + i) : 0;
                memory.write(tlsBase + i, b);
            }
        }
        memory.writeMemoryWord(tebBase, (int) tlsBase);
        memory.writeMemoryWord(tebBase + 4, (int) tlsBase);
        memory.writeMemoryWord(tebBase + 8, (int) stackTop);

        for (int reg = 14; reg <= 31; reg++) {
            long slot = stackTop - 152L + (reg - 14) * 8L;
            cpu.writeMemoryDouble(slot, 0L);
        }
        memory.writeMemoryWord(threadExit, 0x48000000);
        cpu.writeMemoryWord(stackTop - 120L, threadExit);
        cpu.writeMemoryWord(stackTop - 8L, threadExit);

        cpu.setMsr(cpu.getMsr() & 0x7FFFFFFFFFFFFFFFL);
        cpu.setGPR(1, stackTop - 112L);
        cpu.setGPR(3, r3Arg & 0xFFFFFFFFL);
        cpu.setGPR(4, r4Arg & 0xFFFFFFFFL);
        cpu.setGPR(5, 0L);
        cpu.setGPR(13, tebBase);
        cpu.setCtr(entryPoint);
        cpu.setLr(threadExit);
        cpu.setPc(entryPoint);

        if (!enginesInitialized) {
            enginesInitialized = true;
            ringBufferInitialized = true;
            displayModeSet = true;
            System.out.println("[TKERNEL] Gráficos forzados a estado inicializado");
        }
        if (ringBufferBase == 0) {
            ringBufferBase = allocatePhysicalBlock(0x10000);
            ringBufferSize = 0x10000;
            System.out.printf("[TKERNEL] RingBuffer ficticio base=%08X size=%08X%n",
                    ringBufferBase, ringBufferSize);
        }

        cpu.resume();
        System.out.printf("[TKERNEL] THREAD %s: PC=%08X r1=%08X r3=%08X r4=%08X r13=%08X TLS=%08X LR=%08X%n",
                module.getName(), entryPoint, stackTop - 112L, r3Arg, r4Arg, tebBase, tlsBase, threadExit);
    }

    public void startRetailThread(CPU cpu, XenonFlashModuleLoader.Module module, long threadExit) {
        boolean isDll = (module.getXex().moduleFlags() & 0x2) != 0;
        long r3 = isDll ? (module.getImage().getImageBase() & 0xFFFFFFFFL) : 0L;
        long r4 = isDll ? 1L : 0L;
        startRetailThread(cpu, module, module.getEntryPoint(), r3, r4, threadExit);
    }

    public PPCSimulator getSimulator() {
        return simulator;
    }

    public Bus getBus() {
        return bus;
    }

    public Memory getMemory() {
        return memory;
    }

    public int getInstalledThunks() {
        return installedThunks;
    }

    public int getUnknownThunks() {
        return unknownThunks;
    }

    public boolean isBootAnimationRequested() {
        return bootAnimationRequested;
    }

    public int installImports(Path manifest, String module) throws IOException {
        if (!Files.isRegularFile(manifest)) {
            throw new IOException("No existe manifiesto de imports: " + manifest);
        }
        int count = 0;
        int unknown = 0;
        try (BufferedReader in = Files.newBufferedReader(manifest, StandardCharsets.UTF_8)) {
            String line = in.readLine();
            while ((line = in.readLine()) != null) {
                String[] c = line.split(",", -1);
                if (c.length < 8 || !module.equalsIgnoreCase(c[0]) || !"xboxkrnl.exe".equalsIgnoreCase(c[1]) || !"01".equals(c[4])) {
                    continue;
                }
                int ordinal = Integer.parseInt(c[3], 16);
                long thunk = Long.parseLong(c[5], 16);
                if (thunk == 0 || (thunk & 3) != 0) {
                    continue;
                }
                importThunkAddresses.put(importKey(module, "xboxkrnl.exe", ordinal), thunk & 0xFFFFFFFFL);
                memory.writeMemoryWord(thunk, PPC_LI_R0_BASE | (ordinal & 0xFFFF));
                memory.writeMemoryWord(thunk + 4, PPC_SC);
                if (!namedKernelOrdinals.containsKey(ordinal)) {
                    unknownThunks++;
                    unknown++;
                }
                installedThunks++;
                count++;
            }
        }
        return count;
    }

    private static String importKey(String module, String library, int ordinal) {
        return module.toLowerCase(Locale.ROOT) + "|" + library.toLowerCase(Locale.ROOT) + "|" + ordinal;
    }

    public long getImportThunk(String module, String library, int ordinal) {
        Long address = importThunkAddresses.get(importKey(module, library, ordinal));
        if (address == null) {
            throw new IllegalStateException("Import thunk no instalado: "
                    + module + " -> " + library + " ordinal " + ordinal);
        }
        return address;
    }

    public void registerBootAnimationModule(XenonPeLoader.LoadedImage image) {
        if (image == null) {
            throw new IllegalArgumentException("Imagen bootanim nula");
        }
        bootAnimationImageBase = image.getImageBase();
        bootAnimationImageSize = image.getSizeOfImage();
        bootAnimationModuleHasEntryPoint = image.hasEntryPoint();
        bootAnimationModuleLoaded = true;
        System.out.printf("[TKERNEL] BOOTANIM REGISTERED base=%08X size=%08X entry=%08X hasEntry=%b%n",
                bootAnimationImageBase, bootAnimationImageSize, image.getEntryPoint(), bootAnimationModuleHasEntryPoint);
    }

    public int registerBootAnimationModule(XenonPeLoader.LoadedImage image,
            Path xexPath, Path extractedResourceDirectory) throws IOException {
        registerBootAnimationModule(image);
        if (xexPath == null || !Files.isRegularFile(xexPath)) {
            throw new IOException("No existe XEX original para leer ResourceInfo: " + xexPath);
        }
        if (extractedResourceDirectory == null || !Files.isDirectory(extractedResourceDirectory)) {
            throw new IOException("No existe directorio de recursos bootanim: " + extractedResourceDirectory);
        }
        XexFile xex = XexFile.parse(Files.readAllBytes(xexPath));
        int mapped = 0;
        for (XexFile.Resource resource : xex.resources()) {
            Path resourceFile = extractedResourceDirectory.resolve(resource.name);
            if (!Files.isRegularFile(resourceFile)) {
                resourceFile = extractedResourceDirectory.resolve(resource.name + ".wav");
            }
            if (!Files.isRegularFile(resourceFile)) {
                throw new IOException("Falta recurso XEX '" + resource.name + "' en " + extractedResourceDirectory);
            }
            byte[] bytes = Files.readAllBytes(resourceFile);
            memory.writeBlock(resource.address, bytes);
            bootAnimationResources.put(resource.name.toLowerCase(Locale.ROOT), resource.address);
            mapped++;
        }
        return mapped;
    }

    public boolean isBootAnimationResourceMapped(String name) {
        return name != null && bootAnimationResources.containsKey(name.toLowerCase(Locale.ROOT));
    }

    public long getBootAnimationResourceAddress(String name) {
        Long address = name == null ? null : bootAnimationResources.get(name.toLowerCase(Locale.ROOT));
        if (address == null) {
            throw new IllegalStateException("Recurso bootanim no mapeado: " + name);
        }
        return address;
    }

    public void installBootAnimationStartupHook(CPU cpu, long hookAddress, long aniStartThunk, long xamEntry, long threadExit) {
        checkGuestAddress(hookAddress, "hookAddress");
        checkGuestAddress(aniStartThunk, "aniStartThunk");
        checkGuestAddress(xamEntry, "xamEntry");
        checkGuestAddress(threadExit, "threadExit");
        int p = 0;
        p = emitLoadAddress(hookAddress, p, 12, aniStartThunk);
        memory.writeMemoryWord(hookAddress + p, 0x7D8903A6);
        p += 4;
        memory.writeMemoryWord(hookAddress + p, 0x4E800421);
        p += 4;
        p = emitLoadAddress(hookAddress, p, 12, threadExit);
        memory.writeMemoryWord(hookAddress + p, 0x7D8803A6);
        p += 4;
        p = emitLoadAddress(hookAddress, p, 12, xamEntry);
        memory.writeMemoryWord(hookAddress + p, 0x7D8903A6);
        p += 4;
        memory.writeMemoryWord(hookAddress + p, 0x4E800420);
        p += 4;
        cpu.setMsr(cpu.getMsr() & 0x7FFFFFFFFFFFFFFFL);
        cpu.setGPR(1, 0x9FFFFDB0L);
        cpu.setGPR(3, 1L);
        cpu.setLr(threadExit);
        cpu.writeMemoryWord(0x9FFFFFF8L, (int) threadExit);
        cpu.writeMemoryWord(0x9FFFFF88L, (int) threadExit);
        memory.writeMemoryWord(threadExit, 0x48000000);
        cpu.setPc(hookAddress);
    }

    private int emitLoadAddress(long base, int offset, int reg, long address) {
        int hi = (int) ((address >>> 16) & 0xFFFF);
        int lo = (int) (address & 0xFFFF);
        memory.writeMemoryWord(base + offset, 0x3C000000 | (reg << 21) | hi);
        offset += 4;
        memory.writeMemoryWord(base + offset, 0x60000000 | (reg << 21) | (reg << 16) | lo);
        offset += 4;
        return offset;
    }

    public int installXamImports(Path importManifest, String module, Path exportManifest) throws IOException {
        return 0;
    }

    public void installCompatibilityKernelMirror() {
    }

    public int installXamImportsFromXex(XexFile xex, byte[] peImage,
            String module, Path exportManifest, boolean compatibilityStubs) throws IOException {
        return 0;
    }

    public void startThread(CPU cpu, long routine, long argument, long stackTop, long threadExit, long trampoline) {
        int hi = (int) ((routine >>> 16) & 0xFFFF);
        int lo = (int) (routine & 0xFFFF);
        memory.writeMemoryWord(trampoline, 0x3D800000 | hi);
        memory.writeMemoryWord(trampoline + 4, 0x618C0000 | lo);
        memory.writeMemoryWord(trampoline + 8, 0x7D8903A6);
        memory.writeMemoryWord(trampoline + 12, 0x4E800421);
        memory.writeMemoryWord(trampoline + 16, 0x4E800020);
        cpu.setMsr(cpu.getMsr() & 0x7FFFFFFFFFFFFFFFL);
        cpu.setGPR(1, (stackTop - 0x100L) & ~15L);
        cpu.setGPR(3, argument & 0xFFFFFFFFL);
        cpu.setCtr(routine);
        cpu.setLr(threadExit & 0xFFFFFFFFL);
        cpu.setPc(trampoline & 0xFFFFFFFFL);
    }

    private static void checkGuestAddress(long address, String field) {
        if (address <= 0 || address > 0xFFFFFFFFL || (address & 3L) != 0) {
            throw new IllegalArgumentException(field + " fuera de rango/alineación PPC: "
                    + Long.toHexString(address));
        }
    }

    public void initializeBootDisplay() {
        enginesInitialized = true;
        ringBufferInitialized = true;
        displayModeSet = true;
    }

    // Crea un objeto de hilo ficticio de 0x200 bytes y lo registra en guestThreadObjects
    private long createThreadObject(int handle) {
        long obj = allocatePhysicalBlock(0x200);
        memory.writeMemoryWord(obj + 0x00, 0);          // ExitStatus = 0
        memory.writeMemoryWord(obj + 0x04, 1);          // SignalState = 1 (señalado)
        memory.writeMemoryWord(obj + 0x120, 0);         // ExitStatus real
        guestThreadObjects.put(handle, obj);
        return obj;
    }

    public long invokeKernelExport(int ordinal, CPU cpu) {
        String name = namedKernelOrdinals.get(ordinal);
        if (name == null) {
            return unsigned(STATUS_NOT_IMPLEMENTED);
        }

        switch (name) {
            case "AniStartBootAnimation": {
                if (!bootAnimationModuleLoaded) {
                    return unsigned(STATUS_NOT_IMPLEMENTED);
                }
                bootAnimationRequested = true;
                initializeBootDisplay();
                Long animEntry = getModuleExport("bootanim.xex", 1);
                System.out.printf("[TKERNEL] AniStartBootAnimation: module=%08X export1=%08X%n",
                        bootAnimationImageBase, animEntry == null ? 0L : animEntry);
                if (animEntry != null && animEntry != 0 && bootAnimationModule != null) {
                    CPU animCpu = simulator.spawnGuestThread(false);
                    long animExit = 0x9FFC0030L;
                    startRetailThread(animCpu, bootAnimationModule, animEntry, 1L, 0L, animExit);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "ExGetXConfigSetting":
                return exGetXConfigSetting(cpu);
            case "ExRegisterTitleTerminateNotification":
                return unsigned(STATUS_SUCCESS);
            case "ExAllocatePool":
            case "ExAllocatePoolWithTag":
            case "ExAllocatePoolTypeWithTag": {
                long bytes = u32(cpu.getGPRVal(3));
                return allocatePhysicalBlock(bytes == 0 ? 0x1000L : bytes);
            }
            case "ExFreePool":
                return 0L;
            case "ExCreateMutant": {
                long handleOut = u32(cpu.getGPRVal(3));
                int handle = allocateGuestHandle();
                if (handleOut != 0) {
                    memory.writeMemoryWord(handleOut, handle);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "ExCreateThread":
                return exCreateThread(cpu);
            case "ExTerminateThread":
                cpu.halt();
                return unsigned(STATUS_SUCCESS);
            case "KeInitializeEvent": {
                long eventPtr = u32(cpu.getGPRVal(3));
                int type = (int) u32(cpu.getGPRVal(4));
                int state = (int) u32(cpu.getGPRVal(5));
                if (eventPtr != 0) {
                    memory.writeMemoryWord(eventPtr, type);
                    memory.writeMemoryWord(eventPtr + 4, state);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "KeSetEvent": {
                long eventPtr = u32(cpu.getGPRVal(3));
                int prev = 0;
                if (eventPtr != 0) {
                    prev = memory.readMemoryWord(eventPtr + 4);
                    memory.writeMemoryWord(eventPtr + 4, 1);
                }
                return prev;
            }

            case "VdSetSurface_Stub":
            case "VdSetDisplayMode_Stub":
            case "VdSetDisplayModeOverride_Stub":
                return unsigned(STATUS_SUCCESS);

            case "NtQueryInformationProcess_Stub":
            case "XamNotifyCreateListener_Stub":
                return unsigned(STATUS_SUCCESS);
            case "KeDelayExecutionThread": {
                long interval = u32(cpu.getGPRVal(3));
                cpu.setTimeBase(cpu.getTimeBase() + Math.max(1000L, interval));
                return unsigned(STATUS_SUCCESS);
            }
            case "KeGetCurrentProcessType":
                return 1L;
            case "KeQueryPerformanceCounter":
            case "NtQueryPerformanceCounter":
                return cpu.getTimeBase();
            case "KeQueryPerformanceFrequency":
                return 50_000_000L;
            case "KeQuerySystemTime": {
                long outPtr = u32(cpu.getGPRVal(3));
                if (outPtr != 0) {
                    long ft = 0x01DC000000000000L + cpu.getTimeBase();
                    memory.writeMemoryWord(outPtr, (int) (ft >>> 32));
                    memory.writeMemoryWord(outPtr + 4, (int) ft);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "KeTlsAlloc":
                synchronized (tlsSlotsInUse) {
                    for (int i = 0; i < tlsSlotsInUse.length; i++) {
                        if (!tlsSlotsInUse[i]) {
                            tlsSlotsInUse[i] = true;
                            return i;
                        }
                    }
                }
                return 0xFFFFFFFFL;
            case "KeTlsFree": {
                int idx = (int) u32(cpu.getGPRVal(3));
                if (idx >= 0 && idx < tlsSlotsInUse.length) {
                    synchronized (tlsSlotsInUse) {
                        tlsSlotsInUse[idx] = false;
                    }
                    return 1L;
                }
                return 0L;
            }
            case "KeTlsGetValue": {
                int idx = (int) u32(cpu.getGPRVal(3));
                long[] vals = threadTlsValues.get(cpu);
                if (vals != null && idx >= 0 && idx < vals.length) {
                    return vals[idx] & 0xFFFFFFFFL;
                }
                return 0L;
            }
            case "KeTlsSetValue": {
                int idx = (int) u32(cpu.getGPRVal(3));
                long val = u32(cpu.getGPRVal(4));
                if (idx >= 0 && idx < tlsSlotsInUse.length) {
                    long[] vals = threadTlsValues.computeIfAbsent(cpu, k -> new long[tlsSlotsInUse.length]);
                    vals[idx] = val;
                    return 1L;
                }
                return 0L;
            }
            case "RtlRaiseStatus_Stub":
                cpu.setPc(cpu.getLr());
                return unsigned(STATUS_SUCCESS);
            case "NtQueryVolumeInformationFile_Stub": {
                long infoPtr = u32(cpu.getGPRVal(5));
                long length = u32(cpu.getGPRVal(6));
                long ioStatus = u32(cpu.getGPRVal(4));
                if (infoPtr != 0 && length >= 0x18) {
                    for (long i = 0; i < length && i < 0x40; i++) {
                        memory.write(infoPtr + i, (byte) 0);
                    }
                    memory.writeMemoryWord(infoPtr + 8, 0x58415446);
                    memory.writeMemoryWord(infoPtr + 12, 0x00000001);
                    memory.writeMemoryWord(infoPtr + 16, 0);
                    memory.writeMemoryWord(infoPtr + 20, 0);
                }
                writeIoStatus(ioStatus, STATUS_SUCCESS, (int) length);
                return unsigned(STATUS_SUCCESS);
            }
            case "PsCreateSystemThreadEx_Stub": {
                long handleOut = u32(cpu.getGPRVal(3));
                long threadIdOut = u32(cpu.getGPRVal(6));
                long startContext1 = u32(cpu.getGPRVal(7));
                long startContext2 = u32(cpu.getGPRVal(8));
                long startRoutine = u32(cpu.getGPRVal(10));

                int handle = allocateGuestHandle();
                if (handleOut != 0) {
                    memory.writeMemoryWord(handleOut, handle);
                }
                if (threadIdOut != 0) {
                    memory.writeMemoryWord(threadIdOut, handle);
                }
                createThreadObject(handle);

                // Rutina dummy que retorna (blr). Se escribe una vez.
                memory.writeMemoryWord(0x9FFB0000L, 0x4E800020);

                boolean validCode = (startRoutine >= 0x82000000L && startRoutine < 0x92000000L);
                long effectiveRoutine = validCode ? startRoutine : 0x9FFB0000L;

                CPU child = simulator.spawnGuestThread(false);
                long regionBase = nextThreadRegion;
                nextThreadRegion -= 0x40000L;
                if (nextThreadRegion < 0x82000000L) {
                    nextThreadRegion = 0x83F00000L;
                }
                long stackTop = (regionBase + 0x38000L) & ~15L;
                long tebBase = (regionBase + 0x39000L) & ~15L;
                long tlsBase = (regionBase + 0x3A000L) & ~15L;
                memory.writeMemoryWord(tebBase, (int) tlsBase);
                memory.writeMemoryWord(tebBase + 4, (int) tlsBase);

                long exitTrap = 0x9FFC0060L;
                memory.writeMemoryWord(exitTrap, 0x4E800020); // blr

                child.setMsr(child.getMsr() & 0x7FFFFFFFFFFFFFFFL);
                child.setGPR(1, stackTop - 0x100L);
                child.setGPR(3, startContext1);
                child.setGPR(4, startContext2);
                child.setGPR(13, tebBase);
                child.setCtr(effectiveRoutine);
                child.setLr(exitTrap);
                child.setPc(effectiveRoutine);

                guestThreads.put(handle, child);
                System.out.printf("[TKERNEL] PsCreateSystemThreadEx entry=%08X ctx1=%08X ctx2=%08X handle=%08X%s%n",
                        effectiveRoutine, startContext1, startContext2, handle,
                        validCode ? "" : " (dummy blr)");
                return unsigned(STATUS_SUCCESS);
            }
            case "Unknown405_Stub": {
                long outPtr = u32(cpu.getGPRVal(3));
                if (outPtr != 0) {
                    int fakeHandle = allocateGuestHandle();
                    memory.writeMemoryWord(outPtr, fakeHandle);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "NtCreateFile":
                return ntCreateFile(cpu);
            case "NtOpenFile":
                return ntOpenFile(cpu);
            case "NtReadFile":
                return ntReadFile(cpu);
            case "NtQueryInformationFile":
                return ntQueryInformationFile(cpu);
            case "NtClose": {
                int handle = (int) u32(cpu.getGPRVal(3));
                guestFiles.remove(handle);
                guestThreads.remove(handle);
                guestThreadObjects.remove(handle);
                return unsigned(STATUS_SUCCESS);
            }
            case "MmAllocatePhysicalMemory":
            case "MmAllocatePhysicalMemoryEx": {
                long size = u32(cpu.getGPRVal(4));
                return allocatePhysicalBlock(size == 0 ? 0x1000L : size);
            }
            case "MmFreePhysicalMemory":
                return 0L;
            case "MmGetPhysicalAddress":
                return u32(cpu.getGPRVal(3)) & 0x1FFFFFFFL;
            case "MmQueryAllocationSize":
                return 0x1000L;
            case "NtAllocateVirtualMemory":
                return ntAllocateVirtualMemory(cpu);
            case "NtFreeVirtualMemory":
            case "NtProtectVirtualMemory":
            case "NtQueryVirtualMemory":
                return unsigned(STATUS_SUCCESS);
            case "NtResumeThread": {
                int handle = (int) u32(cpu.getGPRVal(3));
                long suspendCountPtr = u32(cpu.getGPRVal(4));
                if (suspendCountPtr != 0) {
                    memory.writeMemoryWord(suspendCountPtr, 1);
                }
                CPU target = guestThreads.get(handle);
                if (target != null) {
                    target.resume();
                } else if (!guestThreadObjects.containsKey(handle)) {
                    // Handle inexistente: registrar objeto ficticio para que no explote
                    createThreadObject(handle);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "KeWaitForSingleObject": {
                // Bloqueo real: si el objeto es un hilo, esperar a que termine
                long objectPtr = u32(cpu.getGPRVal(3));
                if (objectPtr == 0) {
                    return unsigned(STATUS_SUCCESS);
                }
                // Si el objeto está señalado (SignalState != 0), retornar de inmediato
                int signalState = memory.readMemoryWord(objectPtr + 4);
                if (signalState != 0) {
                    return unsigned(STATUS_SUCCESS);
                }
                // Si no, timeout: retornar STATUS_TIMEOUT para que Aurora no se cuelgue
                return unsigned(STATUS_TIMEOUT);
            }
            case "RtlInitAnsiString": {
                long dest = u32(cpu.getGPRVal(3));
                long src = u32(cpu.getGPRVal(4));
                int len = 0;
                if (src != 0) {
                    while (len < 0x1000 && memory.read(src + len) != 0) {
                        len++;
                    }
                }
                if (dest != 0) {
                    memory.writeMemoryWord(dest, (len << 16) | (src != 0 ? (len + 1) : 0));
                    memory.writeMemoryWord(dest + 4, (int) src);
                }
                return 0L;
            }
            case "RtlInitUnicodeString": {
                long dest = u32(cpu.getGPRVal(3));
                long src = u32(cpu.getGPRVal(4));
                int bytes = 0;
                if (src != 0) {
                    while (bytes < 0x1000) {
                        int ch = ((memory.read(src + bytes) & 0xFF) << 8) | (memory.read(src + bytes + 1) & 0xFF);
                        if (ch == 0) {
                            break;
                        }
                        bytes += 2;
                    }
                }
                if (dest != 0) {
                    memory.writeMemoryWord(dest, (bytes << 16) | (src != 0 ? (bytes + 2) : 0));
                    memory.writeMemoryWord(dest + 4, (int) src);
                }
                return 0L;
            }
            case "NtOpenSymbolicLinkObject": {
                long handleOut = u32(cpu.getGPRVal(3));
                if (handleOut != 0) {
                    int h = allocateGuestHandle();
                    memory.writeMemoryWord(handleOut, h);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "ObReferenceObjectByHandle": {
                int handle = (int) u32(cpu.getGPRVal(3));
                long objectOut = u32(cpu.getGPRVal(5));
                if (handle == 0) {
                    return unsigned(STATUS_INVALID_HANDLE); // ← handle nulo: fallar, no fingir
                }
                if (objectOut == 0) {
                    return unsigned(STATUS_INVALID_PARAMETER);
                }
                Long obj = guestThreadObjects.get(handle);
                if (obj == null) {
                    obj = createThreadObject(handle);
                }
                memory.writeMemoryWord(objectOut, (int) (obj & 0xFFFFFFFFL));
                return unsigned(STATUS_SUCCESS);
            }
            case "NtCreateDirectoryObject_Stub":
            case "NtOpenDirectoryObject_Stub":
            case "KeInitializeSemaphore":
            case "KeInitializeSpinLock":
            case "ExCreateEvent":
            case "NtSetInformationThread":
            case "NtQueryInformationThread":
            case "XexGetModuleSection":
            case "XexGetProcedureAddress":
            case "XamBuildResourceLocator":
            case "XamEnumerate":
            case "XamGetSystemVersion":
            case "XamTaskSchedule":
            case "KeSetBasePriorityThread":
            case "KeSetPriorityThread":
            case "KeSetAffinityThread":
            case "KePulseEvent":
            case "KeResumeThread":
            case "KeSetTimer":
            case "KeSetTimerEx":
            case "KeSetTimerEx_Alias":
            case "KeRaiseIrqlToDpcLevel":
            case "RtlRaiseException":
            case "RtlTimeFieldsToTime":
            case "RtlTimeToTimeFields":
            case "XMAReleaseContext":
                return unsigned(STATUS_SUCCESS);
            case "KeGetCurrentThread":
                return cpu.getGPRVal(13);
            case "RtlInitializeCriticalSection":
            case "RtlInitializeCriticalSectionAndSpinCount": {
                long csPtr = u32(cpu.getGPRVal(3));
                if (csPtr != 0) {
                    for (int i = 0; i < 28; i += 4) {
                        memory.writeMemoryWord(csPtr + i, 0);
                    }
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "RtlTryEnterCriticalSection":
                return 1L;
            case "RtlEnterCriticalSection":
            case "RtlLeaveCriticalSection":
                return unsigned(STATUS_SUCCESS);
            case "VdGetCurrentDisplayInformation": {
                long infoPtr = u32(cpu.getGPRVal(3));
                if (infoPtr != 0) {
                    memory.writeMemoryWord(infoPtr, 800);
                    memory.writeMemoryWord(infoPtr + 4, 600);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "VdGetCurrentDisplayMode": {
                long wPtr = u32(cpu.getGPRVal(3));
                long hPtr = u32(cpu.getGPRVal(4));
                if (wPtr != 0) {
                    memory.writeMemoryWord(wPtr, 800);
                }
                if (hPtr != 0) {
                    memory.writeMemoryWord(hPtr, 600);
                }
                return unsigned(STATUS_SUCCESS);
            }
            case "VdInitializeEngines":
                enginesInitialized = true;
                return 1L;
            case "VdInitializeRingBuffer": {
                long reqBase = u32(cpu.getGPRVal(3));
                long pages = u32(cpu.getGPRVal(4));
                // Si reqBase no es una dirección válida (por ejemplo, es un status de error),
                // asignar un ring buffer ficticio en memoria válida.
                if (reqBase < 0x82000000L || reqBase > 0x92000000L) {
                    ringBufferBase = allocatePhysicalBlock(pages * 0x1000L);
                    System.out.printf("[TKERNEL] VdInitializeRingBuffer: r3=%08X inválido, usando base ficticia=%08X%n",
                            reqBase, ringBufferBase);
                } else {
                    ringBufferBase = reqBase;
                }
                ringBufferSize = pages * 0x1000L;
                ringBufferInitialized = true;
                return 0L;
            }
            case "VdSetDisplayMode":
            case "VdSetDisplayModeOverride":
                displayModeSet = true;
                return unsigned(STATUS_SUCCESS);
            case "VdSwap": {
                long fbBase = 0x10000000L;
                long surfaceAddr = u32(cpu.getGPRVal(4)); // Dirección de la superficie de Aurora

                // Si Aurora nos da una superficie válida, copiarla
                if (surfaceAddr >= 0x80000000L && surfaceAddr < 0xA0000000L) {
                    for (long i = 0; i < 800 * 600 * 4; i += 4) {
                        int pixel = memory.readMemoryWord(surfaceAddr + i);
                        memory.writeMemoryWord(fbBase + i, pixel);
                    }
                    System.out.printf("[TKERNEL] VdSwap: copiada superficie %08X%n", surfaceAddr);
                } else {
                    // Fallback: patrón rojo si no hay superficie válida
                    for (long i = 0; i < 800 * 600; i++) {
                        memory.writeMemoryWord(fbBase + i * 4, 0x00FF0000);
                    }
                }
                simulator.getFrameBuffer().present();
                return 0L;
            }
            case "VdShutdownEngines":
                enginesInitialized = false;
                return unsigned(STATUS_SUCCESS);
            case "VdTurnDisplayOff":
                displayModeSet = false;
                return unsigned(STATUS_SUCCESS);
            case "XMACreateContext": {
                long ctxOut = u32(cpu.getGPRVal(3));
                long ctxAddr = allocatePhysicalBlock(0x100);
                if (ctxOut != 0) {
                    memory.writeMemoryWord(ctxOut, (int) ctxAddr);
                }
                return unsigned(STATUS_SUCCESS);
            }
            default:
                if (name.endsWith("_Stub")) {
                    long handleOutPtr = u32(cpu.getGPRVal(3));
                    if (handleOutPtr != 0 && (name.startsWith("NtOpen") || name.startsWith("NtCreate"))) {
                        int fakeHandle = allocateGuestHandle();
                        memory.writeMemoryWord(handleOutPtr, fakeHandle);
                    }
                    return unsigned(STATUS_SUCCESS);
                }
                return unsigned(STATUS_NOT_IMPLEMENTED);
        }
    }

    private long exGetXConfigSetting(CPU cpu) {
        int category = (int) u32(cpu.getGPRVal(3));
        int setting = (int) u32(cpu.getGPRVal(4));
        long buffer = u32(cpu.getGPRVal(5));
        int size = (int) u32(cpu.getGPRVal(6));
        long reqSizePtr = u32(cpu.getGPRVal(7));

        if (size < 0 || size > 0x1000) {
            size = 4;
        }

        if (reqSizePtr != 0) {
            memory.writeMemoryWord(reqSizePtr, size);
        }

        if (buffer != 0 && size > 0) {
            for (int i = 0; i < size; i++) {
                memory.write(buffer + i, (byte) 0);
            }
            if (category == 3 && size >= 4) {
                memory.writeMemoryWord(buffer, 0x00000100);
            } else if (category == 1 && setting == 1 && size >= 4) {
                memory.writeMemoryWord(buffer, 1);
            } else if (size >= 4) {
                memory.writeMemoryWord(buffer, 1);
            }
        }
        return unsigned(STATUS_SUCCESS);
    }

    private long exCreateThread(CPU cpu) {
        long handleOut = u32(cpu.getGPRVal(3));
        long stackSize = u32(cpu.getGPRVal(4));
        long threadIdOut = u32(cpu.getGPRVal(5));
        long xapiStartup = u32(cpu.getGPRVal(6));
        long startRoutine = u32(cpu.getGPRVal(7));
        long startContext = u32(cpu.getGPRVal(8));
        long flags = u32(cpu.getGPRVal(9));

        boolean suspended = (flags & 0x1) != 0;
        CPU child = simulator.spawnGuestThread(suspended);
        long regionBase = nextThreadRegion;
        nextThreadRegion -= 0x40000L;
        if (nextThreadRegion < 0x82000000L) {
            nextThreadRegion = 0x83F00000L;
        }
        long stackTop = (regionBase + 0x38000L) & ~15L;
        long tebBase = (regionBase + 0x39000L) & ~15L;
        long tlsBase = (regionBase + 0x3A000L) & ~15L;
        memory.writeMemoryWord(tebBase, (int) tlsBase);
        memory.writeMemoryWord(tebBase + 4, (int) tlsBase);

        long entry = (xapiStartup > 0x10000L && xapiStartup < 0x82000000L) ? xapiStartup : startRoutine;
        long exitTrap = 0x9FFC0040L;
        memory.writeMemoryWord(exitTrap, 0x48000000);
        boolean validCode = (entry >= 0x80000000L && entry < 0x82000000L) || (entry >= 0x90000000L && entry < 0xA0000000L);
        if (!validCode) {
            entry = exitTrap;
        }
        child.setMsr(child.getMsr() & 0x7FFFFFFFFFFFFFFFL);
        child.setGPR(1, stackTop - 0x100L);
        child.setGPR(3, (xapiStartup > 0x10000L) ? startRoutine : startContext);
        child.setGPR(4, startContext);
        child.setGPR(13, tebBase);
        child.setCtr(entry);
        child.setLr(exitTrap);
        child.setPc(entry);
        int handle = allocateGuestHandle();
        // Registrar siempre el objeto ficticio, incluso si el hilo no es válido
        createThreadObject(handle);
        if (validCode) {
            guestThreads.put(handle, child);
        } else {
            child.halt();
        }
        if (handleOut != 0) {
            memory.writeMemoryWord(handleOut, handle);
        }
        if (threadIdOut != 0) {
            memory.writeMemoryWord(threadIdOut, handle);
        }
        System.out.printf("[TKERNEL] ExCreateThread entry=%08X routine=%08X ctx=%08X suspended=%b handle=%08X%n",
                entry, startRoutine, startContext, suspended, handle);
        return unsigned(STATUS_SUCCESS);
    }

    private synchronized long allocatePhysicalBlock(long size) {
        long aligned = (size + 0xFFFL) & ~0xFFFL;
        if (aligned <= 0) {
            aligned = 0x1000L;
        }
        long addr = nextVirtualAlloc;
        nextVirtualAlloc += aligned;
        for (long i = 0; i < aligned; i += 4) {
            memory.writeMemoryWord(addr + i, 0);
        }
        return addr;
    }

    private long ntAllocateVirtualMemory(CPU cpu) {
        long baseAddrPtr = u32(cpu.getGPRVal(3));
        long regionSizePtr = u32(cpu.getGPRVal(4));
        if (baseAddrPtr == 0 || regionSizePtr == 0) {
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        long reqBase = u32(memory.readMemoryWord(baseAddrPtr));
        long reqSize = u32(memory.readMemoryWord(regionSizePtr));
        long alignedSize = (reqSize + 0xFFFL) & ~0xFFFL;
        if (alignedSize == 0) {
            alignedSize = 0x1000L;
        }
        long allocBase = reqBase != 0 ? (reqBase & ~0xFFFL) : allocatePhysicalBlock(alignedSize);
        memory.writeMemoryWord(baseAddrPtr, (int) allocBase);
        memory.writeMemoryWord(regionSizePtr, (int) alignedSize);
        return unsigned(STATUS_SUCCESS);
    }

    public long startBootAnimation(CPU cpu) {
        bootAnimationRequested = true;
        initializeBootDisplay();
        return 0;
    }

    // En TKernel.java
    private long ntCreateFile(CPU cpu) {
        long handleOut = u32(cpu.getGPRVal(3));
        long ioStatus = u32(cpu.getGPRVal(6));

        // FINGIMOS el archivo
        if (flashFs == null) {
            int fakeHandle = allocateGuestHandle();
            GuestFile fakeFile = new GuestFile(new XenonFlashFs.Entry("", 0, 0, 0));
            guestFiles.put(fakeHandle, fakeFile); // Lo guardamos

            if (handleOut != 0) {
                memory.writeMemoryWord(handleOut, fakeHandle);
            }
            writeIoStatus(ioStatus, STATUS_SUCCESS, 1);
            return unsigned(STATUS_SUCCESS); // Éxito
        }

        long objectAttributes = u32(cpu.getGPRVal(5));
        int disposition = (int) u32(cpu.getGPRVal(10));
        if (disposition != 0 && disposition != FILE_OPEN) {
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        return openFlashObject(handleOut, objectAttributes, ioStatus);
    }

    private long ntOpenFile(CPU cpu) {
        long handleOut = u32(cpu.getGPRVal(3));
        long ioStatus = u32(cpu.getGPRVal(6));

        // FINGIMOS la apertura
        if (flashFs == null) {
            if (handleOut != 0) {
                int fakeHandle = allocateGuestHandle();
                memory.writeMemoryWord(handleOut, fakeHandle);
            }
            writeIoStatus(ioStatus, STATUS_SUCCESS, 1);
            return unsigned(STATUS_SUCCESS);
        }

        long objectAttributes = u32(cpu.getGPRVal(5));
        return openFlashObject(handleOut, objectAttributes, ioStatus);
    }

    private long openFlashObject(long handleOut, long objectAttributes, long ioStatus) {
        if (handleOut == 0 || objectAttributes == 0) {
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        long objectName = u32(memory.readMemoryWord(objectAttributes + 8));
        if (objectName == 0) {
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        int nameLength = (memory.readMemoryWord(objectName) >>> 16) & 0xFFFF;
        if (nameLength == 0) {
            nameLength = memory.readMemoryWord(objectName) & 0xFFFF;
        }
        long nameBuffer = u32(memory.readMemoryWord(objectName + 4));
        if (nameLength <= 0 || nameLength > 0x800 || (nameLength & 1) != 0 || nameBuffer == 0) {
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        StringBuilder path = new StringBuilder(nameLength / 2);
        for (int i = 0; i < nameLength; i += 2) {
            int ch = ((memory.read(nameBuffer + i) & 0xFF) << 8)
                    | (memory.read(nameBuffer + i + 1) & 0xFF);
            path.append((char) ch);
        }
        String normalized = normalizeFlashPath(path.toString());
        XenonFlashFs.Entry entry = findFlashEntry(normalized);
        int handle = allocateGuestHandle(); // Reutilizado de allocateGuestHandle()
        if (entry == null) {
            GuestFile fakeFile = new GuestFile(new XenonFlashFs.Entry("", 0, 0, 0));
            guestFiles.put(handle, fakeFile);
            memory.writeMemoryWord(handleOut, handle);
            writeIoStatus(ioStatus, STATUS_SUCCESS, 1);
            System.out.printf("[TKERNEL] OpenFile '%s' -> handle=%08X (fake, no encontrado)%n",
                    path, handle);
            return unsigned(STATUS_SUCCESS);
        }
        guestFiles.put(handle, new GuestFile(entry));
        memory.writeMemoryWord(handleOut, handle);
        writeIoStatus(ioStatus, STATUS_SUCCESS, 1);
        System.out.printf("[TKERNEL] OpenFile '%s' -> handle=%08X size=%08X%n",
                path, handle, entry.size);
        return unsigned(STATUS_SUCCESS);
    }

    private long ntReadFile(CPU cpu) {
        int handle = (int) u32(cpu.getGPRVal(3));
        long ioStatus = u32(cpu.getGPRVal(7));
        long buffer = u32(cpu.getGPRVal(8));
        long length = u32(cpu.getGPRVal(9));
        long byteOffsetPtr = u32(cpu.getGPRVal(10));
        GuestFile file = guestFiles.get(handle);
        if (file == null) {
            writeIoStatus(ioStatus, STATUS_INVALID_HANDLE, 0);
            return unsigned(STATUS_INVALID_HANDLE);
        }
        if (buffer == 0 || length > Integer.MAX_VALUE) {
            writeIoStatus(ioStatus, STATUS_INVALID_PARAMETER, 0);
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        long offset = file.position;
        if (byteOffsetPtr != 0) {
            long high = u32(memory.readMemoryWord(byteOffsetPtr));
            long low = u32(memory.readMemoryWord(byteOffsetPtr + 4));
            offset = (high << 32) | low;
        }

        if (file.entry.size == 0) {
            for (long i = 0; i < length; i++) {
                memory.write(buffer + i, (byte) 0);
            }
            // Agregamos firma básica para evadir otras comprobaciones si leen cabeceras
            if (offset == 0 && length >= 4) {
                memory.writeMemoryWord(buffer, 0x58455832); // Firma "XEX2" 
            }
            // rellenamos con una pequeña firma o patrón seguro
            for (long i = 4; i < length; i++) {
                memory.write(buffer + i, (byte) ((i * 31 + 17) & 0xFF));
            }
            writeIoStatus(ioStatus, STATUS_SUCCESS, (int) length);
            file.position = offset + length;
            return unsigned(STATUS_SUCCESS);
        }

        if (offset < 0 || offset > file.entry.size) {
            writeIoStatus(ioStatus, STATUS_INVALID_PARAMETER, 0);
            return unsigned(STATUS_INVALID_PARAMETER);
        }
        int count = (int) Math.min(length, file.entry.size - offset);
        try {
            if (count > 0) {
                memory.writeBlock(buffer, flashFs.read(file.entry, offset, count));
            }
        } catch (IOException ex) {
            writeIoStatus(ioStatus, STATUS_OBJECT_NAME_NOT_FOUND, 0);
            return unsigned(STATUS_OBJECT_NAME_NOT_FOUND);
        }
        file.position = offset + count;
        writeIoStatus(ioStatus, STATUS_SUCCESS, count);
        return unsigned(STATUS_SUCCESS);
    }

    private long ntQueryInformationFile(CPU cpu) {
        int handle = (int) u32(cpu.getGPRVal(3));
        long ioStatus = u32(cpu.getGPRVal(4));
        long infoBuffer = u32(cpu.getGPRVal(5));
        long length = u32(cpu.getGPRVal(6));
        int infoClass = (int) u32(cpu.getGPRVal(7));

        GuestFile file = guestFiles.get(handle);

        if (infoBuffer != 0 && length > 0 && length <= 0x1000) {
            for (int i = 0; i < length; i++) {
                memory.write(infoBuffer + i, (byte) 0);
            }
            long fileSize = (file != null) ? file.entry.size : 0L; // Retornamos 0 de tamaño si es nulo
            if (infoClass == 5 && length >= 24) {
                memory.writeMemoryWord(infoBuffer + 8, (int) (fileSize >>> 32));
                memory.writeMemoryWord(infoBuffer + 12, (int) fileSize);
            } else if (infoClass == 34 && length >= 56) {
                memory.writeMemoryWord(infoBuffer + 40, (int) (fileSize >>> 32));
                memory.writeMemoryWord(infoBuffer + 44, (int) fileSize);
            }
        }
        writeIoStatus(ioStatus, STATUS_SUCCESS, (int) length);
        return unsigned(STATUS_SUCCESS);
    }

    private synchronized int allocateGuestHandle() {
        int start = nextGuestHandle;
        do {
            int candidate = nextGuestHandle;
            nextGuestHandle += 4;
            if (nextGuestHandle <= 0 || nextGuestHandle > 0x7FFFFFF0) {
                nextGuestHandle = 0x100;
            }
            if (!guestFiles.containsKey(candidate) && !guestThreads.containsKey(candidate)
                    && !guestThreadObjects.containsKey(candidate)) {
                return candidate;
            }
        } while (nextGuestHandle != start);
        throw new IllegalStateException("Tabla de handles guest agotada");
    }

    private void writeIoStatus(long address, int status, int information) {
        if (address == 0) {
            return;
        }
        memory.writeMemoryWord(address, status);
        memory.writeMemoryWord(address + 4, information);
    }

    private XenonFlashFs.Entry findFlashEntry(String normalizedPath) {
        XenonFlashFs.Entry exact = flashFs.find(normalizedPath);
        if (exact != null) {
            return exact;
        }
        String base = normalizedPath;
        int slash = base.lastIndexOf('/');
        if (slash >= 0) {
            base = base.substring(slash + 1);
        }
        exact = flashFs.find(base);
        if (exact != null) {
            return exact;
        }
        for (XenonFlashFs.Entry entry : flashFs.entries()) {
            String candidate = normalizeFlashPath(entry.name);
            if (normalizedPath.endsWith("/" + candidate)) {
                return entry;
            }
        }
        return null;
    }

    private static String normalizeFlashPath(String value) {
        String path = value.replace('\\', '/').toLowerCase(Locale.ROOT);
        while (path.startsWith("/")) {
            path = path.substring(1);
        }
        while (path.contains("//")) {
            path = path.replace("//", "/");
        }
        return path;
    }

    private static long u32(long value) {
        return value & 0xFFFFFFFFL;
    }

    private static long unsigned(int value) {
        return value & 0xFFFFFFFFL;
    }
}
