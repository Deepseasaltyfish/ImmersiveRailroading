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
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

/**
 * 车辆持久化。与 WorldData 同模式：纯文件 + TagCompound，
 * 不依赖 MC 的 WorldSavedData。
 *
 * 目录：<levelDirectory>/immersiverailroading<dimId>/vehicles.dat
 * 内容：{ vehicles: { <uuid>: <serializeToTagCompound() 结果> } }
 */
public class VehiclePersistence {
    private static final String FILENAME = "vehicles.dat";

    private static File fileFor(File levelDirectory, World world) {
        File dir = new File(levelDirectory, "immersiverailroading" + world.getId());
        return new File(dir, FILENAME);
    }

    /** 挂到 CommonEvents.WorldData.SAVE */
    public static void save(World world, File levelDirectory) {
        if (!world.isServer) return;
        if (!Config.ConfigDebug.enableEntityPool) return; // 新增

        OfflineEntityPool pool = OfflineEntityPool.get(world);
        Map<UUID, TagCompound> snapshot = new HashMap<>();
        for (ModdedEntity e : pool.getAllEntities()) {
            snapshot.put(e.getPersistentID(), e.serializeToTagCompound());
        }

        TagCompound root = new TagCompound();
        root.setMap("vehicles", snapshot, UUID::toString, tc -> tc);

        try {
            File file = fileFor(levelDirectory, world);
            File parent = file.getParentFile();
            if (parent != null && !parent.exists() && !parent.mkdirs()) {
                ModCore.warn("Unable to create vehicle data directory %s", parent);
            }
            Files.write(file.toPath(), root.toBytes());
            ImmersiveRailroading.info("[POOL] saved %d vehicles to %s", snapshot.size(), file);
        } catch (IOException e) {
            ModCore.catching(e, "Failed to save vehicles for world %d", world.getId());
        }
    }

    /** 挂到 CommonEvents.WorldData.LOAD */
    public static void load(World world, File levelDirectory) {
        if (!world.isServer) return;
        if (!Config.ConfigDebug.enableEntityPool) return; // 新增

        File file = fileFor(levelDirectory, world);
        if (!file.exists()) return;

        try {
            byte[] bytes = Files.readAllBytes(file.toPath());
            TagCompound root = new TagCompound(bytes);
            if (!root.hasKey("vehicles")) return;

            Map<UUID, TagCompound> vehicles = root.getMap("vehicles", UUID::fromString, tc -> tc);
            OfflineEntityPool pool = OfflineEntityPool.get(world);

            int restored = 0, skipped = 0;
            for (Map.Entry<UUID, TagCompound> entry : vehicles.entrySet()) {
                if (pool.getEntity(entry.getKey()) != null) {
                    // MC 已经恢复了这个 UUID（或其它的路径先到），跳过
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
            ImmersiveRailroading.info("[POOL] restored %d vehicles (%d skipped) from %s",
                    restored, skipped, file);
        } catch (IOException e) {
            ModCore.catching(e, "Failed to load vehicles for world %d", world.getId());
        }
    }
}