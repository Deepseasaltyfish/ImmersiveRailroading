package cam72cam.immersiverailroading.entity;

import cam72cam.immersiverailroading.Config;
import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.mod.ModCore;
import cam72cam.mod.entity.CustomEntity;
import cam72cam.mod.entity.ModdedEntity;
import cam72cam.mod.event.CommonEvents;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.serialization.TagCompound;
import cam72cam.mod.world.World;

import java.util.*;
import java.util.stream.Collectors;

public class OfflineEntityPool {
    /** 测试期打开，会打印状态切换日志。发布前关掉。 */
    private static final boolean DEBUG_LOG = true;
    /** 高频 tick 日志单独开关，避免 DEBUG_LOG 打开后刷屏。 */
    private static final boolean DEBUG_TICK = false;

    private static final Map<World, OfflineEntityPool> INSTANCES = new HashMap<>();
    private static boolean registered = false;

    // P0 固定阈值，P1 接 Config + 滞后
    private static final double SPAWN_DISTANCE_SQ   = 128.0 * 128.0;
    private static final double DESPAWN_DISTANCE_SQ = 160.0 * 160.0;
    private static final int PROXIMITY_INTERVAL = 20;

    private final World world;
    private final Map<UUID, ModdedEntity> all = new HashMap<>();
    private final Set<UUID> activelyDespawning = new HashSet<>();
    private int tickCounter = 0;

    private OfflineEntityPool(World world) {
        this.world = world;
    }

    public static OfflineEntityPool get(World world) {
        return INSTANCES.computeIfAbsent(world, OfflineEntityPool::new);
    }

    public static void register() {
        if (registered) return;
        registered = true;

        World.registerLifecycleListener(new World.EntityLifecycleListener() {
            @Override
            public void onEntityAdded(net.minecraft.entity.Entity entity) {
                if (!(entity instanceof ModdedEntity)) return;
                World w = World.get(entity.world);
                if (w == null || !w.isServer) return;
                UUID uuid = entity.getPersistentID();
                OfflineEntityPool pool = get(w);

                ModdedEntity current = pool.all.get(uuid);
                if (current == entity) {
                    // spawn 触发（池里就是这个实例），什么都不做
                    return;
                }
                if (current != null) {
                    // 不同实例（MC 从 chunk 恢复的野实体），丢弃
                    log("onEntityAdded: reject MC duplicate %s", uuid);
                    entity.setDead();
                    return;
                }
                pool.add((ModdedEntity) entity);
            }

            @Override
            public void onEntityRemoved(net.minecraft.entity.Entity entity) {
                if (!(entity instanceof ModdedEntity)) return;
                World w = World.get(entity.world);
                if (w == null || !w.isServer) return;
                UUID uuid = entity.getPersistentID();
                OfflineEntityPool pool = get(w);

                if (pool.activelyDespawning.contains(uuid)) return;

                ModdedEntity current = pool.all.get(uuid);
                if (current != null && current != entity) {
                    // 被 setDead 的野实体，池里的有效实例不动
                    log("onEntityRemoved: ignore discarded duplicate %s", uuid);
                    return;
                }

                if (entity.isDead) {
                    pool.remove(uuid);
                }
            }
        });

        World.registerOfflineEntityLookup(uuid -> {
            if (!Config.ConfigDebug.enableEntityPool) return null;

            for (OfflineEntityPool pool : INSTANCES.values()) {
                ModdedEntity e = pool.all.get(uuid);
                if (e != null) {
                    CustomEntity self = e.getSelf();
                    if (self != null) return self;
                }
            }
            return null;
        });

        World.onTick(OfflineEntityPool::onWorldTick);
        CommonEvents.World.UNLOAD.subscribe(w -> {
            World wrapped = World.get(w);
            if (wrapped != null) {
                OfflineEntityPool removed = INSTANCES.remove(wrapped);
                if (removed != null) log("unload pool for world %d (%d entities)", wrapped.getId(), removed.all.size());
            }
        });
    }

    private static void onWorldTick(World w) {
        if (!w.isServer) return;

        if (!Config.ConfigDebug.enableEntityPool) {
            OfflineEntityPool pool = INSTANCES.get(w);
            if (pool != null) pool.flushAndDisable();
            return;
        }

        OfflineEntityPool pool = INSTANCES.get(w);
        if (pool != null) pool.tick();
    }

    // todo 需要注意的是，不再池子开启的时候放置的车是不会主动进池子的，目前没加专门的机制把他们捞进来
    public void flushAndDisable() {
        if (all.isEmpty()) return;
        ImmersiveRailroading.info("[POOL] Disabling pool, flushing %d vehicles back to world...", all.size());
        for (ModdedEntity e : new ArrayList<>(all.values())) {
            e.isDead = false;
            if (!e.addedToChunk) {
                // 1. 原生同步加载区块，确保实体有地方落座
                e.world.getChunk(e.chunkCoordX, e.chunkCoordZ);

                // 2. 顺手向 ChunkManager 申请 2 秒的强加载，防止它塞进去后光速被卸载
                cam72cam.mod.world.World wrappedWorld = cam72cam.mod.world.World.get(e.world);
                if (wrappedWorld != null) {
                    wrappedWorld.keepLoaded(new cam72cam.mod.math.Vec3i(e.chunkCoordX << 4, 0, e.chunkCoordZ << 4));
                }

                // 3. 塞进世界，触发 tracker.join 和 chunk.entityLists
                e.world.spawnEntity(e);
            }
        }
        all.clear();
        activelyDespawning.clear();
        ImmersiveRailroading.info("[POOL] Flush complete.");
    }

    // ───────────────────────── 池操作 ─────────────────────────

    private void add(ModdedEntity e) {
        if (!Config.ConfigDebug.enableEntityPool) return;

        UUID uuid = e.getPersistentID();
        ModdedEntity existing = all.put(uuid, e);
        if (existing == e) return;  // 重复 add，不打印
        if (existing != null) {
            log("add: replace %s (old=%s new=%s)", uuid, existing, e);
        } else {
            log("add %s (size=%d)", uuid, all.size());
        }
    }

    private void remove(UUID uuid) {
        if (!Config.ConfigDebug.enableEntityPool) return;

        if (all.remove(uuid) != null) {
            log("remove %s (size=%d)", uuid, all.size());
        }
    }

    private void tick() {
        if (!Config.ConfigDebug.enableEntityPool) return;

        // 1. 手动 tick 所有离线实体
        for (ModdedEntity e : all.values()) {
            if (!e.addedToChunk && !e.isDead) {
                if (DEBUG_TICK) log("tick offline %s @ %s", e.getPersistentID(), e.getPositionVector());
                try {
                    e.onUpdate();
                } catch (Exception ex) {
                    ModCore.catching(ex, "Offline tick failed for %s", e.getPersistentID());
                }
            }
        }

        // 2. 定期扫描距离
        if (++tickCounter >= PROXIMITY_INTERVAL) {
            tickCounter = 0;
            try {
                checkProximity();
            } catch (Exception ex) {
                ModCore.catching(ex, "checkProximity failed in world %d", world.getId());
            }
        }
    }

    private void checkProximity() {
        List<cam72cam.mod.entity.Player> players = world.getEntities(cam72cam.mod.entity.Player.class);
        if (players.isEmpty()) return;

        for (ModdedEntity e : all.values()) {
            if (e.isDead) continue;

            CustomEntity self = e.getSelf();
            if (self != null && !self.getPassengers().isEmpty()) continue;

            double distSq = nearestPlayerDistSq(players, new Vec3d(e.getPositionVector()));

            if (e.addedToChunk) {
                if (distSq > DESPAWN_DISTANCE_SQ) despawn(e, distSq);
            } else {
                if (distSq < SPAWN_DISTANCE_SQ) spawn(e, distSq);
            }
        }
    }

    private double nearestPlayerDistSq(List<cam72cam.mod.entity.Player> players, Vec3d pos) {
        double best = Double.MAX_VALUE;
        for (cam72cam.mod.entity.Player p : players) {
            double d = p.internal.getPositionVector().squareDistanceTo(pos.internal());
            if (d < best) best = d;
        }
        return best;
    }

    private void despawn(ModdedEntity e, double distSq) {
        UUID uuid = e.getPersistentID();
        log("despawn %s @ %s (distSq=%.0f)", uuid, e.getPositionVector(), distSq);
        activelyDespawning.add(uuid);
        try {
            world.internal.removeEntityDangerously(e);

            // 兜底：removeEntityDangerously 里 chunk 为 null 时 addedToChunk 不会被清
            if (e.addedToChunk) {
                log("despawn fixup: %s still addedToChunk, force clear", uuid);
                e.addedToChunk = false;
                world.internal.loadedEntityList.remove(e);
            }

            e.isDead = false;   // 池继续 tick 它
        } finally {
            activelyDespawning.remove(uuid);
        }
    }

    private void spawn(ModdedEntity e, double distSq) {
        UUID uuid = e.getPersistentID();
        log("spawn %s @ %s (distSq=%.0f) addedToChunk=%s isDead=%s",
                uuid, e.getPositionVector(), distSq, e.addedToChunk, e.isDead);
        e.isDead = false;
        if (!e.addedToChunk) {
            world.internal.spawnEntity(e);   // 内部 addEntity + onEntityAdded + tracker.join
        }
    }

    // ───────────────────────── 查询 ─────────────────────────

    public Collection<ModdedEntity> getAllEntities() {
        return all.values();
    }

    public ModdedEntity getEntity(UUID uuid) {
        if (!Config.ConfigDebug.enableEntityPool) return null;

        return all.get(uuid);
    }

    /** 供 Simulation 用：在线 + 离线全部编组车辆 */
    public List<EntityCoupleableRollingStock> getAllRollingStock() {
        if (!Config.ConfigDebug.enableEntityPool) return Collections.emptyList();

        return all.values().stream()
                .map(ModdedEntity::getSelf)
                .filter(s -> s instanceof EntityCoupleableRollingStock)
                .map(s -> (EntityCoupleableRollingStock) s)
                .collect(Collectors.toList());
    }

    /** 供调试命令用：一行摘要 */
    public String getStats() {
        int online = 0, offline = 0;
        for (ModdedEntity e : all.values()) {
            if (e.addedToChunk) online++; else offline++;
        }
        return String.format("pool=%d online=%d offline=%d activelyDespawning=%d",
                all.size(), online, offline, activelyDespawning.size());
    }

    private static void log(String fmt, Object... args) {
        if (DEBUG_LOG) ImmersiveRailroading.info("[POOL] " + fmt, args);
    }

    public ModdedEntity createFromTagCompound(TagCompound data) {
        ModdedEntity e = world.createModdedEntity();
        e.deserializeFromTagCompound(data);
        e.addedToChunk = false;
        e.isDead = false;
        add(e);
        return e;
    }
}