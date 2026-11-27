package cam72cam.immersiverailroading.entity.physics;

import cam72cam.immersiverailroading.Config;
import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.immersiverailroading.entity.EntityCoupleableRollingStock;
import cam72cam.immersiverailroading.entity.physics.chrono.ChronoState;
import cam72cam.immersiverailroading.entity.physics.chrono.ServerChronoState;
import cam72cam.immersiverailroading.net.MRSSyncPacket;
import cam72cam.immersiverailroading.physics.TickPos;
import cam72cam.mod.entity.Player;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.util.*;
import java.util.stream.Collectors;

public class Simulation {

    public static boolean forceQuickUpdates = false;
    public static int calculatedStates;
    public static int restStates;
    public static int keptStates;

    double maxCouplerDist = 4;

    private final World world;
    private final int startTickID;
    List<Map<UUID, SimulationState>> stateMaps;
    private final List<Vec3i> blocksAlreadyBroken;
    List<EntityCoupleableRollingStock> loaded;

    /** TEMP diagnostics for the 1.21 "train splits after chunk/entity reload" bug. Set false to silence. */
    public static boolean debugCouplers = true;
    /**
     * Master switch for the NON-CORE 1.21 workarounds: cached-body bridging, "healed"
     * couplers, coupler geometry sanity clamps and teleport-sized move corrections.
     * The core fix (rebuild from persisted TickPos + rebase onto the session timeline)
     * does not depend on any of them; keep this false to verify that on its own.
     */
    public static boolean workarounds = false;
    /** Keep a coupler when its partner is merely absent/unregistered. Core behaviour, not a workaround. */
    public static final boolean keepCouplerWhenAbsent = true;
    /** How long a car may stay missing (chunk/entity reload) before its couplers are released. */
    private static final int MISSING_STATE_GRACE_TICKS = 600;
    /** Upper bound on how far a rebuilt state is marched forward. */
    private static final int MAX_STATE_ADVANCE_TICKS = 40;

    private final Set<String> debugOnce = new HashSet<>();
    private static final Map<World, Map<UUID, EntityCoupleableRollingStock>> LAST_LOADED = new WeakHashMap<>();
    private static final Map<World, Map<UUID, CachedState>> LAST_STATES = new WeakHashMap<>();
    /** Partner uuids that have never produced a state in this world -> first time noticed missing. */
    private static final Map<World, Map<UUID, Long>> UNSEEN_MISSING = new WeakHashMap<>();

    private static final class CachedState {
        final SimulationState state;
        final long tick;

        CachedState(SimulationState state, long tick) {
            this.state = state;
            this.tick = tick;
        }
    }

    private void debugOnce(String key, String fmt, Object... args) {
        if (!debugCouplers || !debugOnce.add(key)) {
            return;
        }
        ImmersiveRailroading.info("[IRDBG] sim=" + startTickID + " " + fmt, args);
    }

    private static String describeStock(EntityCoupleableRollingStock stock) {
        if (stock == null) {
            return "null";
        }
        TickPos tp = stock.getTickPos();
        World stockWorld = stock.getWorld();
        return "uuid=" + stock.getUUID()
                + " entityPos=" + stock.getPosition()
                + " tickPos=" + (tp == null ? "null" : tp.tickID + "@" + tp.position)
                + " states=" + stock.states.size()
                + " positions=" + stock.positions.size()
                + " posTicks=" + tickPosRange(stock)
                + " id=" + stock.getId()
                + " chunkLoaded=" + (stockWorld != null && stockWorld.isBlockLoaded(stock.getBlockPosition()))
                + " dead=" + stock.isDead()
                + " coupled=" + stock.getCoupledUUID(EntityCoupleableRollingStock.CouplerType.FRONT)
                + "/" + stock.getCoupledUUID(EntityCoupleableRollingStock.CouplerType.BACK);
    }

    private static String tickPosRange(EntityCoupleableRollingStock stock) {
        if (stock.positions.isEmpty()) {
            return "none";
        }
        int min = Integer.MAX_VALUE;
        int max = Integer.MIN_VALUE;
        for (TickPos pos : stock.positions) {
            min = Math.min(min, pos.tickID);
            max = Math.max(max, pos.tickID);
        }
        return min + ".." + max;
    }

    /**
     * {@code states} is transient while {@code positions} is serialized.  On a 1.21
     * unload/reload the carrier can come back with fresh positions from NBT but the old
     * state objects still attached.  Mixing those two timelines produces a consist whose
     * tickIDs line up while its positions are tens of blocks apart.
     */
    private static boolean statesMatchPositions(EntityCoupleableRollingStock stock) {
        if (stock.states.isEmpty()) {
            return true;
        }
        if (stock.positions.isEmpty()) {
            return false;
        }
        for (SimulationState state : stock.states) {
            TickPos matching = null;
            for (TickPos position : stock.positions) {
                if (position.tickID == state.tickID) {
                    matching = position;
                    break;
                }
            }
            if (matching == null || matching.position == null || state.position == null) {
                return false;
            }
            if (matching.position.distanceToSquared(state.position) > 0.0001) {
                return false;
            }
        }
        return true;
    }

    private void logLoadedChanges() {
        if (!debugCouplers) {
            return;
        }
        Map<UUID, EntityCoupleableRollingStock> previous = LAST_LOADED.get(world);
        Map<UUID, EntityCoupleableRollingStock> current = new HashMap<>();
        for (EntityCoupleableRollingStock stock : loaded) {
            current.put(stock.getUUID(), stock);
        }
        if (previous != null) {
            for (Map.Entry<UUID, EntityCoupleableRollingStock> entry : current.entrySet()) {
                EntityCoupleableRollingStock old = previous.get(entry.getKey());
                if (old == null) {
                    ImmersiveRailroading.info("[IRDBG] sim=%s LOADED + %s", startTickID, describeStock(entry.getValue()));
                } else if (old != entry.getValue()) {
                    ImmersiveRailroading.info("[IRDBG] sim=%s LOADED INSTANCE SWAPPED uuid=%s old=%s new=%s",
                            startTickID, entry.getKey(), describeStock(old), describeStock(entry.getValue()));
                }
            }
            for (Map.Entry<UUID, EntityCoupleableRollingStock> entry : previous.entrySet()) {
                if (!current.containsKey(entry.getKey())) {
                    ImmersiveRailroading.info("[IRDBG] sim=%s LOADED - %s", startTickID, describeStock(entry.getValue()));
                }
            }
        }
        LAST_LOADED.put(world, current);
    }

    private void rememberStates() {
        if (!workarounds) {
            return;
        }
        Map<UUID, CachedState> previous = LAST_STATES.get(world);
        Map<UUID, CachedState> states = new HashMap<>();
        long now = world.getTicks();

        /*
         * Anchor the cache at this pass's CURRENT tick, never at the furthest predicted tick.
         * The future states are up to physicsFutureTicks ahead (several blocks of travel);
         * seeding a car with one of those teleports it and instantly trips the distance check.
         *
         * Keep the cache age tied to the last time the real entity was seen.  A bridged
         * (missing) car also has a state in stateMaps, and refreshing its timestamp every
         * pass would keep a genuinely removed car alive forever.
         */
        Set<UUID> present = new HashSet<>();
        if (loaded != null) {
            for (EntityCoupleableRollingStock stock : loaded) {
                present.add(stock.getUUID());
            }
        }
        Map<UUID, SimulationState> current = stateMaps.isEmpty() ? new HashMap<>() : stateMaps.get(0);
        for (SimulationState state : current.values()) {
            CachedState previousCached = previous == null ? null : previous.get(state.config.id);
            boolean seen = present.contains(state.config.id);
            if (seen) {
                states.put(state.config.id, new CachedState(state.copyFor(state.tickID), now));
            } else if (previousCached != null && now - previousCached.tick <= MISSING_STATE_GRACE_TICKS) {
                states.put(state.config.id, new CachedState(state.copyFor(state.tickID), previousCached.tick));
            }
        }

        /*
         * Carry still-valid entries forward.  A missing car produces no states at all, so
         * rebuilding the map from `current` alone would drop its entry after one pass and the
         * grace period below would never actually apply.
         */
        if (previous != null) {
            for (Map.Entry<UUID, CachedState> entry : previous.entrySet()) {
                if (states.containsKey(entry.getKey())) {
                    continue;
                }
                CachedState cached = entry.getValue();
                if (now - cached.tick > MISSING_STATE_GRACE_TICKS) {
                    continue;
                }
                states.put(entry.getKey(), cached);
            }
        }
        LAST_STATES.put(world, states);

        Map<UUID, Long> unseen = UNSEEN_MISSING.get(world);
        if (unseen != null) {
            unseen.keySet().removeIf(states::containsKey);
            unseen.entrySet().removeIf(entry -> now - entry.getValue() > MISSING_STATE_GRACE_TICKS);
        }
    }

    /**
     * True when a coupled uuid has never produced a state in this world and was first reported
     * missing less than MISSING_STATE_GRACE_TICKS ago.  This is the "first pass after a world /
     * chunk load" case, where the partner entity may not be registered with UMC yet.
     */
    private boolean unseenWithinGrace(UUID uuid) {
        if (!keepCouplerWhenAbsent) {
            return false;
        }
        Map<UUID, Long> unseen = UNSEEN_MISSING.get(world);
        if (unseen == null) {
            unseen = new HashMap<>();
            UNSEEN_MISSING.put(world, unseen);
        }
        long now = world.getTicks();
        Long first = unseen.get(uuid);
        if (first == null) {
            unseen.put(uuid, now);
            return true;
        }
        if (now - first > MISSING_STATE_GRACE_TICKS) {
            // Self-pruning: rememberStates() is skipped while the workarounds are off.
            unseen.remove(uuid);
            return false;
        }
        return now - first <= MISSING_STATE_GRACE_TICKS;
    }

    /** Cars that are currently absent from the world but still inside their grace window. */
    private Set<UUID> missingWithinGrace() {
        Set<UUID> missing = new HashSet<>();
        if (!workarounds) {
            return missing;
        }
        Map<UUID, CachedState> cachedStates = LAST_STATES.get(world);
        if (cachedStates == null || cachedStates.isEmpty()) {
            return missing;
        }
        Set<UUID> present = new HashSet<>();
        for (EntityCoupleableRollingStock stock : loaded) {
            present.add(stock.getUUID());
        }
        long now = world.getTicks();
        for (Map.Entry<UUID, CachedState> entry : cachedStates.entrySet()) {
            if (!present.contains(entry.getKey()) && now - entry.getValue().tick <= MISSING_STATE_GRACE_TICKS) {
                missing.add(entry.getKey());
            }
        }
        return missing;
    }

    private String cachedInfo(UUID uuid) {
        Map<UUID, CachedState> states = LAST_STATES.get(world);
        CachedState cached = states == null ? null : states.get(uuid);
        if (cached == null) {
            return "noCachedState";
        }
        return "lastSeen=" + (world.getTicks() - cached.tick) + "ticksAgo cachedTick=" + cached.state.tickID
                + " cachedPos=" + cached.state.position;
    }

    private SimulationState cachedState(UUID uuid, int tickID) {
        if (!workarounds) {
            return null;
        }
        Map<UUID, CachedState> states = LAST_STATES.get(world);
        if (states == null) {
            return null;
        }
        CachedState cached = states.get(uuid);
        if (cached == null || world.getTicks() - cached.tick > MISSING_STATE_GRACE_TICKS) {
            return null;
        }
        SimulationState state = cached.state.copyFor(cached.state.tickID);
        if (state.tickID < tickID) {
            int advance = Math.min(tickID - state.tickID, MAX_STATE_ADVANCE_TICKS);
            List<Vec3i> broken = new ArrayList<>();
            for (int i = 0; i < advance; i++) {
                state = state.next(state.velocity, broken);
                broken.clear();
            }
        }
        state.tickID = tickID;
        state.dirty = true;
        state.bridged = true;
        return state;
    }


    public Simulation(World world) {
        calculatedStates = keptStates = restStates = 0;
        long startTimeMs = System.currentTimeMillis();
        this.world = world;
        this.startTickID = ((ServerChronoState)ChronoState.getState(world)).getServerTickID();
        // Anchor the persisted TickPos timeline of this session before any stock is rebuilt.
        SimulationState.noteSessionStart(world, startTickID);

        stateMaps = new ArrayList<>();
        blocksAlreadyBroken = new ArrayList<>();

        for (int i = 0; i < Config.ConfigDebug.physicsFutureTicks; i++) {
            stateMaps.add(new HashMap<>());
        }

        for (int i = 0; i < stateMaps.size()-1; i++) {
            simulateTick(i);
        }

        boolean sendPackets = world.getTicks() % (Config.ConfigDebug.physicsFutureTicks / 2) == 0;
        double syncDistanceSq = ImmersiveRailroading.ENTITY_SYNC_DISTANCE * ImmersiveRailroading.ENTITY_SYNC_DISTANCE;
        List<Player> players = sendPackets ? world.getEntities(Player.class) : null;

        // Apply new states
        for (EntityCoupleableRollingStock stock : loaded) {
            stock.states = stateMaps.stream().map(m -> m.get(stock.getUUID())).filter(Objects::nonNull).collect(Collectors.toList());
            for (SimulationState state : stock.states) {
                state.dirty = false;
                // Recovery is a property of this pass only; it must not leak into the next pass.
                state.bridged = false;
            }
            stock.positions = stock.states.stream().map(TickPos::new).collect(Collectors.toList());
            SimulationState latest = stateMaps.get(0).get(stock.getUUID());
            Vec3d stockPos = latest != null ? latest.position : stock.getPosition();
            if (sendPackets && players.stream().anyMatch(player -> player.getPosition().distanceToSquared(stockPos) < syncDistanceSq)) {
                new MRSSyncPacket(stock, stock.positions).sendToObserving(stock);
            }
        }

        logLoadedChanges();
        rememberStates();

        long totalTimeMs = System.currentTimeMillis() - startTimeMs;
        if (totalTimeMs > Config.ConfigDebug.physicsWarnTotalThresholdMs) {
            ImmersiveRailroading.warn("Calculating Immersive Railroading Physics took %sms (%s, %s, %s)", totalTimeMs, calculatedStates, restStates, keptStates);
        }
    }

    public void simulateTick(int iteration) {
        long startTimeMs = System.currentTimeMillis();

        int tickID = startTickID + iteration;

        Map<UUID, SimulationState> stateMap = stateMaps.get(iteration);
        Map<UUID, SimulationState> nextStateMap = stateMaps.get(iteration+1);

        // Should really only ever run once, maybe twice.  Cut it off after 10 times just to be safe
        for (int tryLoad = 0; tryLoad < 10; tryLoad++) {
            // If this is too slow, we could hang onto the list of stock that is at rest.
            // We should only need to do this when stock is no longer at rest or changes dirty flag
            int lastCount = loaded == null ? 0 : loaded.size();
            loaded = new ArrayList<>();
            Set<UUID> loadedUuids = new HashSet<>();
            for (EntityCoupleableRollingStock stock : world.getEntities(EntityCoupleableRollingStock.class)) {
                if (stock.isDead()) {
                    continue;
                }
                if (loadedUuids.add(stock.getUUID())) {
                    loaded.add(stock);
                } else {
                    debugOnce("dupe-" + stock.getUUID() + "-" + tickID,
                            "DUPLICATE wrapped stock uuid=%s (ignored) %s", stock.getUUID(), describeStock(stock));
                }
            }
            boolean newChunksLoaded = lastCount != loaded.size();

            for (EntityCoupleableRollingStock stock : loaded) {

                if (!statesMatchPositions(stock)) {
                    debugOnce("stale-states-" + stock.getUUID() + "-" + tickID,
                            "STALE STATES cleared: states and persisted positions disagree %s", describeStock(stock));
                    stock.states.clear();
                }

                if (!stateMap.containsKey(stock.getUUID())) {
                    if (stock.states.isEmpty()) {
                        /*
                         * The transient states list is lost whenever the carrier entity is
                         * re-created (1.21 does that on every chunk unload/reload).  The persisted
                         * TickPos list survives, so rebuild from it and march it forward to this
                         * tick instead of trusting the NBT/re-created entity position.
                         */
                        SimulationState rebuilt = SimulationState.forTick(stock, tickID);
                        if (!rebuilt.usedTickPos) {
                            SimulationState cached = cachedState(stock.getUUID(), tickID);
                            if (cached != null) {
                                rebuilt = cached;
                            }
                        }
                        /*
                         * Any state that was not reconstructed from an exact TickPos for this tick is
                         * only an approximation.  In 1.21 cars become visible one chunk at a time, so
                         * the approximated body can be a few ticks behind the rest of the consist.
                         * Do not allow that approximation to trip the coupler distance check.
                         */
                        if (!SimulationState.hasExactTickPos(stock, tickID)) {
                            rebuilt.bridged = true;
                        }
                        rebuilt.update(stock);
                        stateMap.put(stock.getUUID(), rebuilt);
                        debugOnce("rebuild-" + stock.getUUID() + "-" + tickID,
                                "REBUILT STATE tick=%s usedTickPos=%s fromTick=%s bridged=%s %s",
                                tickID, rebuilt.usedTickPos, rebuilt.tickPosTickID, rebuilt.bridged, describeStock(stock));
                    } else {
                        for (SimulationState state : stock.states) {
                            int stateIteration = state.tickID - tickID;
                            if (stateIteration >= 0 && stateIteration < stateMaps.size()) {
                                state.update(stock);
                                stateMaps.get(stateIteration).put(stock.getUUID(), state);
                            }
                        }

                        if (!stateMap.containsKey(stock.getUUID())) {
                            // This should only ever happen right after stock is placed.
                            SimulationState state = SimulationState.forTick(stock, tickID);
                            state.bridged = true;
                            stateMap.put(stock.getUUID(), state);
                        }
                    }
                }

                SimulationState state = stateMap.get(stock.getUUID());


                // Don't need to load
                if (state.atRest && !state.dirty) {
                    continue;
                }

                // Keep it loaded
                world.keepLoaded(new Vec3i(state.position));

                if (state.consist.positions == null) {
                    continue;
                }

                // Load interacting stock locations
                // We could in the future do a HashSet<Consist> and reduce the calls to the world.
                for (Vec3i pos : state.consist.positions) {
                    if (!world.isBlockLoaded(pos)) {
                        ImmersiveRailroading.debug("Loading chunk at position %s", pos);
                        // Load the chunk, entities should be directly injected into the world
                        world.getBlock(pos);
                        newChunksLoaded = true;
                    }
                }
            }
            if (!newChunksLoaded) {
                break;
            }
        }

        Set<UUID> missing = missingWithinGrace();
        List<SimulationState> states = new ArrayList<>(stateMap.values());

        // Propagate dirty (even across links that will be since that can change between iterations)
        Set<UUID> dirty = new HashSet<>();
        for (SimulationState state : nextStateMap.values()) { // I don't remember why this is the next map
            if (state.dirty) {
                dirty.addAll(state.consist.ids);
            }
        }
        for (UUID uuid : dirty) {
            SimulationState state = stateMap.get(uuid);
            if (state != null) {
                state.dirty = true;
            }
        }

        // Decouple / fix coupler positions
        for (SimulationState state : states) {
            for (boolean isMyCouplerFront : new boolean[]{true, false}) {
                UUID myID = state.config.id;
                UUID otherID = isMyCouplerFront ? state.interactingFront : state.interactingRear;
                Vec3d myCouplerPos = isMyCouplerFront ? state.couplerPositionFront : state.couplerPositionRear;
                String myCouplerLabel = isMyCouplerFront ? "Front" : "Rear";

                if (otherID == null) {
                    // No existing coupler, nothing to check
                    continue;
                }
                SimulationState other = stateMap.get(otherID);
                if (other == null) {
                    /*
                     * A missing state in this pass does NOT mean the car was removed.  In 1.21 the
                     * carrier ModdedEntity is destroyed/re-created when its chunk unloads, and UMC's
                     * world entity registry can transiently forget it.  The original code treated
                     * "no state this pass" as "removed" and cleared the coupler, which is exactly
                     * what tears a consist into pieces when a player leaves and re-enters the area.
                     */
                    EntityCoupleableRollingStock otherStock = world.getEntity(otherID, EntityCoupleableRollingStock.class);
                    if (otherStock != null && !otherStock.isDead()) {
                        SimulationState bridged = null;
                        for (SimulationState cached : otherStock.states) {
                            if (cached.tickID == state.tickID) {
                                bridged = cached;
                                break;
                            }
                        }
                        if (bridged == null) {
                            // Rebuild from the persisted TickPos list (accurate) or, as a last
                            // resort, from the entity itself.  Flag it so it cannot trip the
                            // distance check while it settles.
                            bridged = SimulationState.forTick(otherStock, state.tickID);
                            bridged.bridged = true;
                        }
                        bridged.update(otherStock);
                        bridged.dirty = true;
                        stateMap.put(otherID, bridged);
                        if (loaded.stream().noneMatch(x -> x.getUUID().equals(otherStock.getUUID()))) {
                            loaded.add(otherStock);
                        }
                        debugOnce("bridge-" + myID + "-" + otherID + "-" + state.tickID,
                                "BRIDGED missing state %s (%s) -> %s fromTick=%s %s",
                                myID, myCouplerLabel, otherID, bridged.tickPosTickID, describeStock(otherStock));
                        continue;
                    }

                    if (workarounds && missing.contains(otherID)) {
                        /*
                         * The car is gone from the world, but we saw it very recently.  Keep the
                         * coupling and feed its last known body into this pass.  Without the
                         * bridge body the loaded cars keep moving while the absent car stays
                         * frozen, and the resulting gap is what tears the consist apart when it
                         * is finally re-added.  The bridge is flagged so it cannot trip the
                         * distance check, and Consist will pull/push it back into formation.
                         */
                        SimulationState cached = cachedState(otherID, state.tickID);
                        if (cached != null) {
                            stateMap.put(otherID, cached);
                        }
                        debugOnce("keep-" + myID + "-" + otherID + "-" + state.tickID,
                                "KEEP COUPLER across missing car %s (%s) -> %s %s",
                                myID, myCouplerLabel, otherID, cachedInfo(otherID));
                        continue;
                    }

                    EntityCoupleableRollingStock meStock = loaded.stream()
                            .filter(x -> x.getUUID().equals(myID)).findFirst()
                            .orElseGet(() -> world.getEntity(myID, EntityCoupleableRollingStock.class));
                    Vec3i otherPos = meStock == null ? null
                            : (isMyCouplerFront ? meStock.lastKnownFront : meStock.lastKnownRear);

                    if (otherPos == null) {
                        /*
                         * No state, no cached body and no last known position.  In 1.21 this is
                         * usually a car whose chunk has not been streamed back in yet, not a
                         * removed coupler.  Treating "no information" as "removed" is what
                         * fragments the train, so keep the link and wait for the car to load.
                         */
                        debugOnce("unknown-" + myID + "-" + otherID + "-" + state.tickID,
                                "KEEP COUPLER (partner not loaded yet) %s (%s) -> %s",
                                myID, myCouplerLabel, otherID);
                        continue;
                    }

                    if (!world.isBlockLoaded(otherPos)) {
                        // Other location is not loaded, we must not need to do this check.
                        debugOnce("unloaded-" + myID + "-" + otherID + "-" + state.tickID,
                                "SKIP DECOUPLE (lastKnown chunk unloaded) %s (%s) -> %s at %s",
                                myID, myCouplerLabel, otherID, otherPos);
                        continue;
                    }

                    if (unseenWithinGrace(otherID)) {
                        // First pass after a load: the partner may simply not be registered yet.
                        debugOnce("keep-unseen-" + myID + "-" + otherID + "-" + state.tickID,
                                "KEEP COUPLER (partner never seen yet) %s (%s) -> %s lastKnown=%s",
                                myID, myCouplerLabel, otherID, otherPos);
                        continue;
                    }

                    // This should really only be hit when removing a piece of stock
                    debugOnce("notfound-" + myID + "-" + otherID + "-" + state.tickID,
                            "DECOUPLE stock-not-found %s (%s) -> %s otherEntity=null lastKnown=%s me=%s",
                            myID, myCouplerLabel, otherID, otherPos,
                            loaded.stream().filter(x -> x.getUUID().equals(myID)).findFirst().map(Simulation::describeStock).orElse("null"));
                    if (isMyCouplerFront) {
                        state.interactingFront = null;
                    } else {
                        state.interactingRear = null;
                    }
                    state.dirty = true;
                    continue;
                }

                boolean isOtherCouplerFront;
                if (myID.equals(other.interactingFront)) {
                    isOtherCouplerFront = true;
                } else if (myID.equals(other.interactingRear)) {
                    isOtherCouplerFront = false;
                } else {
                    if (workarounds && (state.bridged || other.bridged)) {
                        /*
                         * One side was reconstructed for this pass, so its persisted coupler
                         * fields may predate this coupling.  Re-point both sides instead of
                         * severing the train.  It is common for only one of the two carrier
                         * entities to have been re-created in 1.21, so repairing only the
                         * reconstructed side leaves the live side pointing at null forever.
                         */
                        boolean otherFront = myCouplerPos.distanceToSquared(other.couplerPositionFront) <
                                myCouplerPos.distanceToSquared(other.couplerPositionRear);
                        UUID otherFrontId = other.interactingFront;
                        UUID otherRearId = other.interactingRear;
                        if (otherFront && otherFrontId != null && !otherFrontId.equals(myID) && otherRearId == null) {
                            otherFront = false;
                        } else if (!otherFront && otherRearId != null && !otherRearId.equals(myID) && otherFrontId == null) {
                            otherFront = true;
                        }
                        if (otherFront) {
                            other.interactingFront = myID;
                        } else {
                            other.interactingRear = myID;
                        }
                        state.dirty = true;
                        other.dirty = true;
                        debugOnce("heal-" + myID + "-" + otherID + "-" + state.tickID,
                                "HEALED bridged coupler %s (%s) -> %s (stateBridged=%s otherBridged=%s)",
                                myID, myCouplerLabel, otherID, state.bridged, other.bridged);
                        continue;
                    }

                    /*
                    This can happen when a piece of stock is marked dirty (discard existing states ex: throttle/brake)
                    and a piece of stock it couples with in the future states has not been marked dirty.  The dirty
                    stock is starting from 0, while the one it has future coupled to in a previous pass (that's english right?)
                    does not know until this re-check that it has desync'd at this point and must generate new states
                    from here on out in this pass.

                    This should be fixed with the smarter "dirty" logic above
                     */
                    ImmersiveRailroading.warn("%s-%s: Mismatched coupler states: %s (%s) -> %s (%s, %s)",
                            startTickID, state.tickID,
                            myID, myCouplerLabel,
                            otherID, other.interactingFront, other.interactingRear);
                    debugOnce("mismatch-" + myID + "-" + otherID + "-" + state.tickID,
                            "DECOUPLE mismatch %s (%s) -> %s (other front/rear=%s/%s) other=%s",
                            myID, myCouplerLabel, otherID, other.interactingFront, other.interactingRear,
                            describeStock(world.getEntity(otherID, EntityCoupleableRollingStock.class)));
                    if (isMyCouplerFront) {
                        state.interactingFront = null;
                    } else {
                        state.interactingRear = null;
                    }
                    state.dirty = true;
                    other.dirty = true;
                    continue;
                }

                Vec3d otherCouplerPos = isOtherCouplerFront ? other.couplerPositionFront : other.couplerPositionRear;
                String otherCouplerLabel = isOtherCouplerFront ? "Front" : "Rear";

                if (workarounds && (state.bridged || other.bridged)) {
                    // One side was rebuilt for this pass only; its body may still be a few ticks
                    // behind.  Never sever the train over that, but only record a real violation
                    // that we chose to ignore (logging every healthy pair floods the log).
                    double maxCouplerDistScaled = maxCouplerDist * state.config.gauge.scale();
                    double bridgedDistSq = myCouplerPos.distanceToSquared(otherCouplerPos);
                    if (bridgedDistSq > maxCouplerDistScaled * maxCouplerDistScaled) {
                        debugOnce("bridgedist-" + myID + "-" + otherID + "-" + state.tickID,
                                "SKIP distance check (bridged) %s (%s) -> %s (%s) dist=%s max=%s my=%s other=%s",
                                myID, myCouplerLabel, otherID, otherCouplerLabel,
                                Math.sqrt(bridgedDistSq), maxCouplerDistScaled, state.position, other.position);
                    }
                    continue;
                }

                double maxCouplerDistScaled = maxCouplerDist * state.config.gauge.scale();
                double distSq = myCouplerPos.distanceToSquared(otherCouplerPos);
                if (distSq > maxCouplerDistScaled * maxCouplerDistScaled) {
                    debugOnce("snap-" + myID + "-" + otherID + "-" + state.tickID,
                            "DECOUPLE distance %s (%s) -> %s (%s) dist=%s max=%s myCoupler=%s otherCoupler=%s myStatePos=%s otherStatePos=%s my=%s other=%s",
                            myID, myCouplerLabel, otherID, otherCouplerLabel,
                            Math.sqrt(distSq), maxCouplerDistScaled, myCouplerPos, otherCouplerPos,
                            state.position, other.position,
                            describeStock(loaded.stream().filter(x -> x.getUUID().equals(myID)).findFirst().orElse(null)),
                            describeStock(world.getEntity(otherID, EntityCoupleableRollingStock.class)));
                    state.dirty = true;
                    other.dirty = true;

                    if (isMyCouplerFront) {
                        state.interactingFront = null;
                    } else {
                        state.interactingRear = null;
                    }

                    if (isOtherCouplerFront) {
                        other.interactingFront = null;
                    } else {
                        other.interactingRear = null;
                    }
                }
            }
        }


        // check for potential couplings and collisions
        for (int sai = 0; sai < states.size()-1; sai++) {
            SimulationState stateA = states.get(sai);
            if (stateA.interactingFront != null && stateA.interactingRear != null) {
                // There's stock in front and behind, can't really hit any other stock here
                continue;
            }

            for (int sbi = sai+1; sbi < states.size(); sbi++) {
                SimulationState stateB = states.get(sbi);
                if (stateA.atRest && stateB.atRest && !stateA.dirty && !stateB.dirty) {
                    continue;
                }
                if (stateB.interactingFront != null && stateB.interactingRear != null) {
                    // There's stock in front and behind, can't really hit any other stock here
                    continue;
                }

                if (stateA.config.gauge != stateB.config.gauge) {
                    // Same gauge required
                    continue;
                }

                double centerDist = stateA.config.length + stateB.config.length;
                if (stateA.position.distanceToSquared(stateB.position) > centerDist * centerDist) {
                    // Too far to reasonably couple
                    continue;
                }

                if (!stateA.bounds.intersects(stateB.bounds)) {
                    // Not close enough to couple
                    continue;
                }

                if (stateB.config.id.equals(stateA.interactingFront) || stateB.config.id.equals(stateA.interactingRear)) {
                    // Already coupled
                    continue;
                }
                if (stateA.config.id.equals(stateB.interactingFront) || stateA.config.id.equals(stateB.interactingRear)) {
                    // Already coupled (double safe check)
                    continue;
                }

                // At this point the stock are colliding / overlapping and we need to do something about it

                /*
                 * 1. |-----a-----| |-----b-----|
                 * 2. |-----a---|=|----b-----|
                 * 3. |---|=a====b|-----|
                 * Keep in mind that we want to make sure that our other coupler might be a better fit
                 */

                // the coupler to target is whichever one the other's center is closest to
                boolean targetACouplerFront =
                        stateA.couplerPositionFront.distanceToSquared(stateB.position) <
                                stateA.couplerPositionRear.distanceToSquared(stateB.position);
                boolean targetBCouplerFront =
                        stateB.couplerPositionFront.distanceToSquared(stateA.position) <
                                stateB.couplerPositionRear.distanceToSquared(stateA.position);

                // Best coupler is already coupled to something
                if ((targetACouplerFront ? stateA.interactingFront : stateA.interactingRear) != null) {
                    continue;
                }
                if ((targetBCouplerFront ? stateB.interactingFront : stateB.interactingRear) != null) {
                    continue;
                }

                // Since bounding boxes can overlap across different tracks (think parallel curves) we need to do
                // a more fine-grained check here
                Vec3d couplerPosA = targetACouplerFront ? stateA.couplerPositionFront : stateA.couplerPositionRear;
                Vec3d couplerPosB = targetBCouplerFront ? stateB.couplerPositionFront : stateB.couplerPositionRear;
                /*
                 * Two cars can have overlapping bounding boxes while their couplers are far apart
                 * (for example while a reloaded car is being pulled back into formation).  Hooking
                 * them up there would immediately trip the decouple distance check and start an
                 * endless couple / un-couple churn, so use the same threshold for both.
                 */
                double maxAttachDistScaled = maxCouplerDist * stateA.config.gauge.scale();
                if (workarounds && couplerPosA.distanceToSquared(couplerPosB) > maxAttachDistScaled * maxAttachDistScaled) {
                    continue;
                }
                // Move coupler pos up to inside the BB (it's at track level by default)
                // This could be optimized further, but it's an infrequent calculation
                couplerPosA = couplerPosA.add(0, stateB.bounds.max().subtract(stateB.bounds.min()).y/2, 0);
                couplerPosB = couplerPosB.add(0, stateA.bounds.max().subtract(stateA.bounds.min()).y/2, 0);
                if (!stateB.bounds.contains(couplerPosA) || !stateA.bounds.contains(couplerPosB)) {
                    // Not actually on the same track, just a BB collision and can be ignored
                    continue;
                }

                stateA.dirty = true;
                stateB.dirty = true;
                ImmersiveRailroading.debug("%s-%s: Coupling %s (%s) to %s (%s)",
                        startTickID, stateA.tickID,
                        stateA.config.id, targetACouplerFront ? "Front" : "Rear",
                        stateB.config.id, targetBCouplerFront ? "Front" : "Rear");

                // Ok, we are clear to proceed!
                if (targetACouplerFront) {
                    stateA.interactingFront = stateB.config.id;
                } else {
                    stateA.interactingRear = stateB.config.id;
                }
                if (targetBCouplerFront) {
                    stateB.interactingFront = stateA.config.id;
                } else {
                    stateB.interactingRear = stateA.config.id;
                }
            }
        }

        // calculate new velocities
        Consist.iterate(stateMap, nextStateMap, blocksAlreadyBroken);

        long totalTimeMs = System.currentTimeMillis() - startTimeMs;
        if (totalTimeMs > Config.ConfigDebug.physicsWarnThresholdMs) {
            ImmersiveRailroading.warn("Calculating Immersive Railroading Physics Iteration took %sms (%s, %s, %s)", totalTimeMs, calculatedStates, restStates, keptStates);
        }
    }






    public static void simulate(World world) {
        // 100KM/h ~= 28m/s which means non-loaded stationary stock may be phased through at that speed
        // I'm OK with that for now
        // We might want to chunk-load ahead of the train just to be safe?

        if (world.getTicks() % 5 != 0) {
            // Only re-check every 5 ticks
            if (!forceQuickUpdates) {
                return;
            }
        } else {
            forceQuickUpdates = false;
        }

        new Simulation(world);
    }

    public static double calculateRoll(double frontRoll, double rearRoll) {//TODO: multi bogey support?
        return (frontRoll + rearRoll) / 2;
    }
}
