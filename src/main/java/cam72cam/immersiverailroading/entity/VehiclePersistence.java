package cam72cam.immersiverailroading.entity;

import cam72cam.immersiverailroading.Config;
import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.mod.ModCore;
import cam72cam.mod.entity.ModdedEntity;
import cam72cam.mod.serialization.TagCompound;
import cam72cam.mod.world.World;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class VehiclePersistence {
    // 匹配 vehicles_16_0.dat
    private static final Pattern FILE_PATTERN = Pattern.compile("^vehicles_(\\d+)_(\\d+)\\.dat$");

    private static final ExecutorService SAVE_EXECUTOR = Executors.newFixedThreadPool(4, r -> {
        Thread t = new Thread(r, "IR-Vehicle-Save");
        t.setDaemon(true);
        return t;
    });

    private static File dirFor(File levelDirectory, World world) {
        return new File(levelDirectory, "immersiverailroading" + world.getId() + "/vehicles");
    }

    private static File fileFor(File levelDirectory, World world, int totalBuckets, int bucketId) {
        return new File(dirFor(levelDirectory, world), "vehicles_" + totalBuckets + "_" + bucketId + ".dat");
    }

    private static int bucketFor(UUID uuid, int bucketCount) {
        return Math.floorMod(uuid.hashCode(), bucketCount);
    }

    /** 挂到 CommonEvents.WorldData.SAVE */
    public static void save(World world, File levelDirectory) {
        if (!world.isServer) return;
        if (!Config.ConfigDebug.enableEntityPool) return;

        OfflineEntityPool pool = OfflineEntityPool.get(world);
        File dir = dirFor(levelDirectory, world);

        int bucketCount = Math.max(1, Config.ConfigDebug.vehicleBucketCount); // 安全保护
        Map<Integer, List<ModdedEntity>> buckets = new HashMap<>();

        for (ModdedEntity e : pool.getAllEntities()) {
            // 如果只保存离线车：if (e.addedToChunk) continue;
            int b = bucketFor(e.getPersistentID(), bucketCount);
            buckets.computeIfAbsent(b, k -> new ArrayList<>()).add(e);
        }

        SAVE_EXECUTOR.submit(() -> {
            try {
                if (!dir.exists() && !dir.mkdirs()) {
                    ModCore.warn("Unable to create vehicle data directory %s", dir);
                    return;
                }

                Set<String> currentValidFiles = new HashSet<>();
                int totalSaved = 0;

                // 1. 写入所有当前桶
                for (Map.Entry<Integer, List<ModdedEntity>> entry : buckets.entrySet()) {
                    int bucketId = entry.getKey();
                    List<ModdedEntity> entities = entry.getValue();

                    TagCompound root = new TagCompound();
                    Map<UUID, TagCompound> map = new HashMap<>();
                    for (ModdedEntity e : entities) {
                        map.put(e.getPersistentID(), e.serializeToTagCompound());
                    }
                    root.setMap("vehicles", map, UUID::toString, tc -> tc);

                    File file = fileFor(levelDirectory, world, bucketCount, bucketId);
                    Files.write(file.toPath(), root.toBytes());
                    currentValidFiles.add(file.getName());
                    totalSaved += map.size();
                }

                // 2. 清理旧的分卷文件（如果 Config 从 16 改成了 32，旧的 vehicles_16_* 要删掉）
                File[] existingFiles = dir.listFiles();
                if (existingFiles != null) {
                    for (File f : existingFiles) {
                        if (FILE_PATTERN.matcher(f.getName()).matches() && !currentValidFiles.contains(f.getName())) {
                            if (f.delete()) {
                                ImmersiveRailroading.info("[POOL] Deleted outdated bucket file %s", f.getName());
                            }
                        }
                    }
                }

                ImmersiveRailroading.info("[POOL] async saved %d vehicles into %d buckets (config: %d)",
                        totalSaved, buckets.size(), bucketCount);
            } catch (Exception e) {
                ModCore.catching(e, "Failed to async save vehicles for world %d", world.getId());
            }
        });
    }

    /** 挂到 CommonEvents.WorldData.LOAD */
    public static void load(World world, File levelDirectory) {
        if (!world.isServer) return;
        if (!Config.ConfigDebug.enableEntityPool) return;

        File dir = dirFor(levelDirectory, world);
        if (!dir.exists()) return;

        OfflineEntityPool pool = OfflineEntityPool.get(world);
        int restored = 0, skipped = 0;
        int loadedFiles = 0;

        File[] files = dir.listFiles();
        if (files == null) return;

        // 扫描目录下所有符合命名规则的 dat 文件
        for (File file : files) {
            Matcher matcher = FILE_PATTERN.matcher(file.getName());
            if (!matcher.matches()) continue;

            // matcher.group(1) 是旧文件里的总桶数，matcher.group(2) 是分卷号
            // 我们只需要按文件读取即可，不需要管它当时分了多少桶
            try {
                byte[] bytes = Files.readAllBytes(file.toPath());
                TagCompound root = new TagCompound(bytes);
                if (!root.hasKey("vehicles")) continue;

                Map<UUID, TagCompound> vehicles = root.getMap("vehicles", UUID::fromString, tc -> tc);
                for (Map.Entry<UUID, TagCompound> entry : vehicles.entrySet()) {
                    if (pool.getEntity(entry.getKey()) != null) {
                        skipped++;
                        continue;
                    }
                    try {
                        pool.createFromTagCompound(entry.getValue());
                        restored++;
                    } catch (Exception e) {
                        ModCore.catching(e, "Failed to restore vehicle %s", entry.getKey());
                    }
                }
                loadedFiles++;
            } catch (IOException e) {
                ModCore.catching(e, "Failed to load vehicle bucket file %s", file.getName());
            }
        }

        ImmersiveRailroading.info("[POOL] restored %d vehicles (%d skipped) from %d files",
                restored, skipped, loadedFiles);
    }
}