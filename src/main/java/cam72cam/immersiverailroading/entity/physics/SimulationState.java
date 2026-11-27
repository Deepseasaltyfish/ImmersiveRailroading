package cam72cam.immersiverailroading.entity.physics;

import cam72cam.immersiverailroading.Config;
import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.immersiverailroading.entity.EntityCoupleableRollingStock;
import cam72cam.immersiverailroading.entity.Locomotive;
import cam72cam.immersiverailroading.entity.Tender;
import cam72cam.immersiverailroading.entity.physics.chrono.ServerChronoState;
import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.library.PhysicalMaterials;
import cam72cam.immersiverailroading.library.TrackItems;
import cam72cam.immersiverailroading.physics.MovementTrack;
import cam72cam.immersiverailroading.physics.TickPos;
import cam72cam.immersiverailroading.thirdparty.trackapi.ITrack;
import cam72cam.immersiverailroading.tile.TileRailBase;
import cam72cam.immersiverailroading.util.BlockUtil;
import cam72cam.immersiverailroading.thirdparty.trackapi.IRPathingData;
import cam72cam.immersiverailroading.util.Speed;
import cam72cam.immersiverailroading.util.VecUtil;
import cam72cam.mod.entity.boundingbox.IBoundingBox;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.util.DegreeFuncs;
import cam72cam.mod.util.FastMath;
import cam72cam.mod.world.World;

import java.util.*;
import java.util.function.Function;

public class SimulationState {
    public int tickID;

    public Vec3d position;
    public double velocity;
    public float yaw;
    public float pitch;
    public float roll;
    public IBoundingBox bounds;

    // Render purposes
    public float yawFront;
    public float yawRear;
    public float rollFront;
    public float rollRear;

    public Vec3d couplerPositionFront;
    public Vec3d couplerPositionRear;
    public UUID interactingFront;
    public UUID interactingRear;

    public float brakePressure;

    public Vec3d recalculatedAt;
    // All positions in the stock bounds
    public List<Vec3i> collidingBlocks;
    // Any track within those bounds
    public List<Vec3i> trackToUpdate;
    // Blocks that the stock would need to break to move
    public List<Vec3i> interferingBlocks;
    // How much force required to break the interfering blocks
    public float interferingResistance;
    // Blocks that were actually broken and need to be removed
    public List<Vec3i> blocksToBreak;

    public double directResistance;

    public Configuration config;
    public boolean dirty = true;
    public boolean atRest = true;
    public double collided;
    public boolean sliding;
    public boolean frontPushing;
    public boolean frontPulling;
    public boolean rearPushing;
    public boolean rearPulling;
    public Consist consist;
    public float slackFrontPercent;
    public float slackRearPercent;
    /** Diagnostics: true when this state was rebuilt from a persisted TickPos entry. */
    public boolean usedTickPos;
    /** Tick that the TickPos used to seed this state came from (-1 when unused). */
    public int tickPosTickID = -1;
    /** True when this state was reconstructed instead of produced by the normal solver. */
    public boolean bridged;

    /**
     * Session tick reference, shared by every stock in a world.
     * <p>
     * {@link ServerChronoState#getServerTickID()} is an in-memory counter that restarts at 0
     * with every server start, while the TickPos entries persisted in entity NBT keep the
     * previous session's numbering.  After a reload the saved timeline therefore looks like
     * it is thousands of ticks "in the future", and findTickPos() has to fall back to the
     * oldest persisted sample.  Cars that stream back in at different ticks all land on that
     * same sample, so a car that appears five ticks after the rest starts five ticks behind
     * it and the couplers stretch.  Rebasing every stock onto the current timeline once
     * fixes the formation regardless of when each car is loaded.
     */
    private static final Map<World, Integer> SESSION_START = new WeakHashMap<>();
    private static final Map<World, Integer> TICK_OFFSET = new WeakHashMap<>();
    private static final Map<World, Set<UUID>> REBASED_STOCK = new WeakHashMap<>();
    /** A timeline within this many ticks of the current session id is already ours. */
    private static final int SESSION_TOLERANCE_TICKS = 64;

    static void noteSessionStart(World world, int startTickID) {
        if (world == null || world.isClient) {
            return;
        }
        synchronized (SESSION_START) {
            SESSION_START.putIfAbsent(world, startTickID);
        }
    }

    private static void rebaseToSession(EntityCoupleableRollingStock stock, int tickID) {
        World world = stock.getWorld();
        if (world == null || world.isClient) {
            return;
        }

        Integer sessionStart;
        Set<UUID> rebased;
        synchronized (SESSION_START) {
            sessionStart = SESSION_START.get(world);
            rebased = REBASED_STOCK.computeIfAbsent(world, k -> new HashSet<>());
        }
        if (sessionStart == null || rebased.contains(stock.getUUID())) {
            return;
        }

        int newest = Integer.MIN_VALUE;
        for (TickPos tickPos : stock.positions) {
            if (tickPos != null) {
                newest = Math.max(newest, tickPos.tickID);
            }
        }
        if (newest == Integer.MIN_VALUE) {
            // Nothing persisted yet - look again once the stock has been simulated.
            return;
        }
        if (Math.abs(newest - tickID) <= SESSION_TOLERANCE_TICKS) {
            rebased.add(stock.getUUID());
            return;
        }

        int offset;
        synchronized (SESSION_START) {
            Integer known = TICK_OFFSET.get(world);
            if (known == null) {
                known = sessionStart - newest;
                TICK_OFFSET.put(world, known);
            }
            offset = known;
        }
        rebased.add(stock.getUUID());
        if (offset != 0) {
            for (TickPos tickPos : stock.positions) {
                if (tickPos != null) {
                    tickPos.tickID += offset;
                }
            }
        }
        ImmersiveRailroading.info("[IRDBG] REBASED timeline %s offset=%s sessionStart=%s newest=%s",
                stock.getUUID(), offset, sessionStart, newest);
    }

    public static class Configuration {
        public UUID id;
        public Gauge gauge;
        public World world;

        public double width;
        public double length;
        public double height;
        public Function<SimulationState, IBoundingBox> bounds;

        public float offsetFront;
        public float offsetRear;

        public boolean couplerEngagedFront;
        public boolean couplerEngagedRear;
        public double couplerDistanceFront;
        public double couplerDistanceRear;
        public double couplerSlackFront;
        public double couplerSlackRear;

        public double massKg;

        public double maximumAdhesionNewtons;
        public double designAdhesionNewtons;
        public double rollingResistanceCoefficient;
        private final Function<List<Vec3i>, Double> directResistanceNewtons;

        // We don't actually want to use this value, it's only for dirty checking
        private double tractiveEffortFactors;
        private Function<Speed, Double> tractiveEffortNewtons;

        public Double desiredBrakePressure;
        public double independentBrakePosition;

        public boolean hasPressureBrake;

        public Configuration(EntityCoupleableRollingStock stock) {
            id = stock.getUUID();
            gauge = stock.gauge;
            world = stock.getWorld();

            width = stock.getDefinition().getWidth(gauge);
            length = stock.getDefinition().getLength(gauge);
            height = stock.getDefinition().getHeight(gauge);
            double pitchOffset = (Math.abs(Math.sin(Math.toRadians(stock.getRotationPitch())) * length));
            bounds = s -> stock.getDefinition().getBounds(s.yaw, gauge)
                    .offset(s.position.add(0, -(s.position.y - Math.floor(s.position.y)) - pitchOffset, 0))
                    .contract(new Vec3d(0, 0, 0.5 * gauge.scale()));
                    //.contract(new Vec3d(0, 0.5 * this.gauge.scale(), 0))
                    //.offset(new Vec3d(0, 0.5 * this.gauge.scale(), 0));

            offsetFront = stock.getDefinition().getBogeyFront(gauge);
            offsetRear = stock.getDefinition().getBogeyRear(gauge);

            couplerEngagedFront = stock.isCouplerEngaged(EntityCoupleableRollingStock.CouplerType.FRONT);
            couplerEngagedRear = stock.isCouplerEngaged(EntityCoupleableRollingStock.CouplerType.BACK);
            couplerDistanceFront = stock.getDefinition().getCouplerPosition(EntityCoupleableRollingStock.CouplerType.FRONT, gauge);
            couplerDistanceRear = -stock.getDefinition().getCouplerPosition(EntityCoupleableRollingStock.CouplerType.BACK, gauge);
            couplerSlackFront = stock.getDefinition().getCouplerSlack(EntityCoupleableRollingStock.CouplerType.FRONT, gauge);
            couplerSlackRear = stock.getDefinition().getCouplerSlack(EntityCoupleableRollingStock.CouplerType.BACK, gauge);

            this.massKg = stock.getWeight();
            // When FuelRequired is false, most of the time the locos are empty.  Work around that here
            double designMassKg = !Config.ConfigBalance.FuelRequired && (stock instanceof Locomotive || stock instanceof Tender) ? massKg : stock.getMaxWeight();

            if (stock instanceof Locomotive) {
                Locomotive locomotive = (Locomotive) stock;
                tractiveEffortNewtons = locomotive::getTractiveEffortNewtons;
                tractiveEffortFactors = locomotive.getThrottle() + (locomotive.getReverser() * 10);
                desiredBrakePressure = (double)locomotive.getTrainBrake();
            } else {
                tractiveEffortNewtons = speed -> 0d;
                tractiveEffortFactors = 0;
                desiredBrakePressure = null;
            }


            double staticFriction = PhysicalMaterials.STEEL.staticFriction(PhysicalMaterials.STEEL);
            this.maximumAdhesionNewtons = massKg * staticFriction * 9.8 * stock.getBrakeAdhesionEfficiency();
            this.designAdhesionNewtons = designMassKg * staticFriction * 9.8 * stock.getBrakeSystemEfficiency();
            this.independentBrakePosition = stock.getIndependentBrake();
            this.directResistanceNewtons = stock::getDirectFrictionNewtons;
            this.hasPressureBrake = stock.getDefinition().hasPressureBrake();

            this.rollingResistanceCoefficient = stock.getDefinition().rollingResistanceCoefficient;
        }

        @Override
        public boolean equals(Object o) {
            if (o instanceof Configuration) {
                Configuration other = (Configuration) o;
                return couplerEngagedFront == other.couplerEngagedFront &&
                        couplerEngagedRear == other.couplerEngagedRear &&
                        Math.abs(tractiveEffortFactors - other.tractiveEffortFactors) < 0.01 &&
                        Math.abs(massKg - other.massKg)/massKg < 0.01 &&
                        (desiredBrakePressure == null || Math.abs(desiredBrakePressure - other.desiredBrakePressure) < 0.001) &&
                        Math.abs(independentBrakePosition - other.independentBrakePosition) < 0.01;
            }
            return false;
        }

        public double tractiveEffortNewtons(Speed speed) {
            return this.tractiveEffortNewtons.apply(speed);
        }
    }

    private static TickPos findTickPos(EntityCoupleableRollingStock stock, int tickID) {
        TickPos best = null;
        TickPos nearestFuture = null;
        for (TickPos tickPos : stock.positions) {
            if (tickPos == null || tickPos.position == null) {
                continue;
            }
            if (tickPos.tickID == tickID) {
                return tickPos;
            }
            if (tickPos.tickID < tickID && (best == null || tickPos.tickID > best.tickID)) {
                best = tickPos;
            }
            if (tickPos.tickID > tickID && (nearestFuture == null || tickPos.tickID < nearestFuture.tickID)) {
                nearestFuture = tickPos;
            }
        }
        if (best != null) {
            return best;
        }
        /*
         * The persisted tick timeline can be entirely in the future when the world/server clock
         * was reset on load (the entity NBT keeps the previous session's tick ids).  There is
         * then no sample at or before tickID.  The nearest future sample is still the
         * authoritative body position - every car of the consist uses the same timeline, so
         * their relative formation is exact.  Falling back to stock.getPosition() here is what
         * used to rebuild cars from stale/last-frame bodies and tear the train apart.
         */
        if (nearestFuture != null) {
            return nearestFuture;
        }
        return stock.getTickPos();
    }

    public static boolean hasExactTickPos(EntityCoupleableRollingStock stock, int tickID) {
        for (TickPos tickPos : stock.positions) {
            if (tickPos.tickID == tickID) {
                return true;
            }
        }
        return false;
    }

    public SimulationState(EntityCoupleableRollingStock stock) {
        this(stock, ServerChronoState.getState(stock.getWorld()).getServerTickID());
    }

    public SimulationState(EntityCoupleableRollingStock stock, int tickID) {
        this.tickID = tickID;

        /*
         * 1.21 destroys/recreates the MC side of a stock entity when its chunk is
         * reloaded.  At that point stock.getPosition() is the stale NBT position
         * (or 0,0,0 for a freshly constructed ModdedEntity), while the IR TickPos
         * list is current and travels with the entity NBT.  Rebuilding a physics
         * state at 0,0,0 makes the 1D consist simulation think the car is hundreds
         * of blocks away, which is what tears the train apart.  Prefer TickPos.
         */
        rebaseToSession(stock, tickID);
        TickPos tickPos = findTickPos(stock, tickID);
        boolean usedTickPos = tickPos != null && tickPos.position != null;
        this.usedTickPos = usedTickPos;
        this.tickPosTickID = usedTickPos ? tickPos.tickID : -1;

        position = usedTickPos ? tickPos.position : stock.getPosition();
        double fallbackVelocity = stock.getVelocity().length() *
                (DegreeFuncs.delta(VecUtil.toWrongYaw(stock.getVelocity()), stock.getRotationYaw()) < 90 ? 1 : -1);
        /*
         * A carrier entity that was just created/placed reports the teleport from its spawn
         * point as getVelocity().  Feeding that into the solver makes the car fly along the
         * track for the whole session, so treat implausible values as "stopped".
         */
        if (Simulation.workarounds && (!Double.isFinite(fallbackVelocity) || Math.abs(fallbackVelocity) > MAX_FALLBACK_VELOCITY)) {
            fallbackVelocity = 0;
        }
        velocity = usedTickPos && tickPos.speed != null ? tickPos.speed.minecraft() : fallbackVelocity;
        yaw = usedTickPos ? tickPos.rotationYaw : stock.getRotationYaw();
        pitch = usedTickPos ? tickPos.rotationPitch : stock.getRotationPitch();
        roll = usedTickPos ? tickPos.rotationRoll : stock.getRotationRoll();

        interactingFront = stock.getCoupledUUID(EntityCoupleableRollingStock.CouplerType.FRONT);
        interactingRear = stock.getCoupledUUID(EntityCoupleableRollingStock.CouplerType.BACK);

        brakePressure = stock.getBrakePressure();

        config = new Configuration(stock);

        bounds = config.bounds.apply(this);

        yawFront = usedTickPos ? tickPos.frontYaw : stock.getFrontYaw();
        yawRear = usedTickPos ? tickPos.rearYaw : stock.getRearYaw();
        rollFront = usedTickPos && tickPos.frontRoll != null ? tickPos.frontRoll : stock.getFrontRoll();
        rollRear = usedTickPos && tickPos.rearRoll != null ? tickPos.rearRoll : stock.getRearRoll();

        recalculatedAt = position;

        calculateCouplerPositions();

        calculateBlockCollisions(Collections.emptyList());
        blocksToBreak = Collections.emptyList();

        consist = stock.consist;

        // If we just placed it, need to adjust it.  Otherwise, it already existed and is just loading in
        dirty = stock.newlyPlaced;
    }

    /** Upper bound on how far a rebuilt state is marched in one go. */
    private static final int MAX_ADVANCE_TICKS = 40;
    /** Entity velocity larger than this is a load artifact, not real motion (blocks/tick). */
    private static final double MAX_FALLBACK_VELOCITY = 16;
    /** A single tick can never legitimately move a car further than this (blocks/tick). */
    private static final double MAX_MOVE_PER_TICK = 12;
    /** A bogie must stay within |distance| + this of where it started (curve/chord allowance). */
    private static final double MOVE_SLOP = 4;
    private static int debugMoveCount;

    private static boolean isFinite(Vec3d pos) {
        return pos != null && Double.isFinite(pos.x) && Double.isFinite(pos.y) && Double.isFinite(pos.z);
    }

    private static void debugMove(String why, SimulationState state, double distance, Object extra) {
        if (!Simulation.debugCouplers || debugMoveCount >= 40) {
            return;
        }
        debugMoveCount++;
        ImmersiveRailroading.info("[IRDBG] %s state=%s tick=%s distance=%s pos=%s extra=%s",
                why, state.config.id, state.tickID, distance, state.position, extra);
    }

    /**
     * Build a state for an arbitrary tick.
     * <p>
     * The transient {@code states} list is lost whenever the carrier entity is re-created
     * (which 1.21 does on every chunk unload/reload), but the {@code positions} list is
     * persisted.  When the requested tick is newer than the newest persisted position, march
     * that position forward with the normal 1D solver instead of trusting the entity's stale
     * NBT position - a stale body is what makes the coupler distance check snap.
     */
    public static SimulationState forTick(EntityCoupleableRollingStock stock, int tickID) {
        SimulationState state = new SimulationState(stock, tickID);
        int fromTick = state.tickPosTickID;
        if (state.usedTickPos && fromTick >= 0 && fromTick < tickID) {
            int advance = Math.min(tickID - fromTick, MAX_ADVANCE_TICKS);
            List<Vec3i> broken = new ArrayList<>();
            for (int i = 0; i < advance; i++) {
                state = state.next(state.velocity, broken);
                broken.clear();
            }
            state.bridged = true;
            state.dirty = true;
            state.tickPosTickID = fromTick;
        }
        state.tickID = tickID;
        return state;
    }

    private SimulationState(SimulationState prev) {
        this.tickID = prev.tickID + 1;
        this.position = prev.position;
        this.velocity = prev.velocity;
        this.yaw = prev.yaw;
        this.pitch = prev.pitch;
        this.roll = prev.roll;

        this.interactingFront = prev.interactingFront;
        this.interactingRear = prev.interactingRear;

        this.brakePressure = prev.brakePressure;

        this.config = prev.config;

        this.bounds = prev.bounds;

        this.yawFront = prev.yawFront;
        this.yawRear = prev.yawRear;
        this.rollFront = prev.rollFront;
        this.rollRear = prev.rollRear;
        couplerPositionFront = prev.couplerPositionFront;
        couplerPositionRear = prev.couplerPositionRear;

        recalculatedAt = prev.recalculatedAt;
        collidingBlocks = prev.collidingBlocks;
        trackToUpdate = prev.trackToUpdate;
        interferingBlocks = prev.interferingBlocks;
        interferingResistance = prev.interferingResistance;
        blocksToBreak = Collections.emptyList();
        directResistance = prev.directResistance;

        slackFrontPercent = prev.slackFrontPercent;
        slackRearPercent = prev.slackRearPercent;

        consist = prev.consist;
        // A state derived from a bridged state is still an approximation for this whole
        // simulation pass.  Dropping the flag here is what let the rebuilt car be checked
        // against the coupler distance one tick after it was pulled back into formation.
        this.bridged = prev.bridged;
    }

    /** Copy for diagnostics / bridging a temporarily missing entity. */
    public SimulationState copyFor(int tickID) {
        SimulationState copy = new SimulationState(this);
        copy.tickID = tickID;
        copy.dirty = this.dirty;
        copy.atRest = this.atRest;
        copy.collided = this.collided;
        copy.sliding = this.sliding;
        copy.frontPushing = this.frontPushing;
        copy.frontPulling = this.frontPulling;
        copy.rearPushing = this.rearPushing;
        copy.rearPulling = this.rearPulling;
        return copy;
    }

    public void calculateCouplerPositions() {
        Vec3d bogeyFront = VecUtil.fromWrongYawPitch(config.offsetFront, yaw, pitch);
        Vec3d bogeyRear = VecUtil.fromWrongYawPitch(config.offsetRear, yaw, pitch);

        Vec3d positionFront = couplerPositionFront = position.add(bogeyFront);
        Vec3d positionRear = couplerPositionRear = position.add(bogeyRear);

        ITrack trackFront = MovementTrack.findTrack(config.world, positionFront, yaw, config.gauge.value());
        ITrack trackRear = MovementTrack.findTrack(config.world, positionRear, yaw, config.gauge.value());

        if (trackFront != null && trackRear != null) {
            Vec3d couplerVecFront = VecUtil.fromWrongYaw(config.couplerDistanceFront - config.offsetFront, yawFront);
            Vec3d couplerVecRear = VecUtil.fromWrongYaw(config.couplerDistanceRear - config.offsetRear, yawRear);

            IRPathingData front = new IRPathingData(positionFront, 0);//Roll is meaningless for coupler
            IRPathingData rear = new IRPathingData(positionRear, 0);
            trackFront.getNextPosition(front, couplerVecFront, config.gauge.value());
            trackRear.getNextPosition(rear, couplerVecRear, config.gauge.value());
            couplerPositionFront = front.getUMCPos();
            couplerPositionRear = rear.getUMCPos();

            /*
             * Same chunk-boundary hazard as moveAlongTrack: a rail lookup can land on a
             * different part of the network and put a coupler hundreds of blocks away from its
             * own body.  Linkage.setup would then read that as the car's length and "correct"
             * the neighbour to the wrong place.  A coupler can only be around its definition's
             * coupler offset from the body, so fall back to the straight-line position.
             */
            double frontLimit = Math.abs(config.couplerDistanceFront - config.offsetFront) + MOVE_SLOP;
            double rearLimit = Math.abs(config.couplerDistanceRear - config.offsetRear) + MOVE_SLOP;
            if (Simulation.workarounds && (!isFinite(couplerPositionFront)
                    || positionFront.distanceToSquared(couplerPositionFront) > frontLimit * frontLimit)) {
                debugMove("REJECTED BOGUS FRONT COUPLER", this, 0, couplerPositionFront);
                couplerPositionFront = position.add(VecUtil.fromWrongYaw(config.couplerDistanceFront, yaw));
            }
            if (Simulation.workarounds && (!isFinite(couplerPositionRear)
                    || positionRear.distanceToSquared(couplerPositionRear) > rearLimit * rearLimit)) {
                debugMove("REJECTED BOGUS REAR COUPLER", this, 0, couplerPositionRear);
                couplerPositionRear = position.add(VecUtil.fromWrongYaw(config.couplerDistanceRear, yaw));
            }
            //couplerPositionFront = couplerPositionFront.subtract(position).normalize().scale(Math.abs(config.couplerDistanceFront)).add(position);
            //couplerPositionRear = couplerPositionRear.subtract(position).normalize().scale(Math.abs(config.couplerDistanceRear)).add(position);
        }
        if (Objects.equals(couplerPositionFront, positionFront)) {
            couplerPositionFront = position.add(VecUtil.fromWrongYaw(config.couplerDistanceFront, yaw));
        }
        if (Objects.equals(couplerPositionRear, positionRear)) {
            couplerPositionRear = position.add(VecUtil.fromWrongYaw(config.couplerDistanceRear, yaw));
        }
    }

    public void calculateBlockCollisions(List<Vec3i> blocksAlreadyBroken) {
        this.collidingBlocks = config.world.blocksInBounds(this.bounds);
        this.trackToUpdate = new ArrayList<>();
        this.interferingBlocks = new ArrayList<>();
        this.interferingResistance = 0;

        for (Vec3i bp : collidingBlocks) {
            if (blocksAlreadyBroken.contains(bp)) {
                continue;
            }

            if (BlockUtil.isIRRail(config.world, bp)) {
                trackToUpdate.add(bp);
            } else {
                if (Config.ConfigDamage.TrainsBreakBlocks
                        && !BlockUtil.isWhitelisted(config.world, bp)
                        && !BlockUtil.isIRRail(config.world, bp.up())) {
                    if (bp.y >= position.y - (position.y % 1)) { // Prevent it from breaking blocks under the pitched train (bb expanded)
                        interferingBlocks.add(bp);
                        interferingResistance += config.world.getBlockHardness(bp);
                    }
                }
            }
        }
    }

    public SimulationState next() {
        SimulationState next = new SimulationState(this);
        next.dirty = false;
        return next;
    }

    public SimulationState next(double distance, List<Vec3i> blocksAlreadyBroken) {
        SimulationState next = new SimulationState(this);
        if (Simulation.workarounds && (!Double.isFinite(distance) || Math.abs(distance) > MAX_MOVE_PER_TICK)) {
            double clamped = Double.isFinite(distance)
                    ? Math.copySign(MAX_MOVE_PER_TICK, distance)
                    : 0;
            debugMove("CLAMPED MOVE", this, distance, clamped);
            distance = clamped;
        }
        next.moveAlongTrack(distance);
        if (this.position.equals(next.position)) {
            next.velocity = 0;
        } else {
            next.calculateCouplerPositions();
            next.bounds = next.config.bounds.apply(next);

            // We will actually break the blocks
            this.blocksToBreak = this.interferingBlocks;
            // We can now ignore those positions for the rest of the simulation
            blocksAlreadyBroken.addAll(this.blocksToBreak);

            // Calculate the next states interference
            double minDist = Math.max(0.5, Math.abs(velocity*4));
            if (next.recalculatedAt.distanceToSquared(next.position) > minDist * minDist) {
                next.calculateBlockCollisions(blocksAlreadyBroken);
                next.recalculatedAt = next.position;
                next.directResistance = config.directResistanceNewtons.apply(trackToUpdate);
            } else {
                // We put off calculating collisions for now
                next.interferingBlocks = Collections.emptyList();
                next.interferingResistance = 0;
            }
        }
        return next;
    }

    public void update(EntityCoupleableRollingStock stock) {
        Configuration oldConfig = config;
        config = new Configuration(stock);
        dirty = dirty || !config.equals(oldConfig);
    }

    private void moveAlongTrack(double distance) {
        Vec3d positionFront = VecUtil.fromWrongYawPitch(config.offsetFront, yaw, pitch).add(position);
        Vec3d positionRear = VecUtil.fromWrongYawPitch(config.offsetRear, yaw, pitch).add(position);

        // Find tracks
        ITrack trackFront = MovementTrack.findTrack(config.world, positionFront, yawFront, config.gauge.value());
        ITrack trackRear = MovementTrack.findTrack(config.world, positionRear, yawRear, config.gauge.value());
        if (trackFront == null || trackRear == null) {
            return;
        }

        boolean isTable = false;
        if (Math.abs(distance) < 0.0001) {
            TileRailBase frontBase = trackFront instanceof TileRailBase ? (TileRailBase) trackFront : null;
            TileRailBase rearBase  = trackRear instanceof TileRailBase ? (TileRailBase) trackRear : null;
            isTable = checkTileType(frontBase, TrackItems.TURNTABLE)
                      || checkTileType(rearBase, TrackItems.TURNTABLE)
                      || checkTileType(frontBase, TrackItems.TRANSFERTABLE)
                      || checkTileType(rearBase, TrackItems.TRANSFERTABLE);
            if (!isTable) {
                return;
            }
        }

        boolean isReversed = distance < 0;

        if (isReversed) {
            distance = -distance;
            yawFront += 180;
            yawRear += 180;
            rollFront = -rollFront;
            rollRear = -rollRear;
            roll = -roll;
        }

        IRPathingData nextFront = new IRPathingData(positionFront, rollFront);
        IRPathingData nextRear = new IRPathingData(positionRear, rollRear);
        trackFront.getNextPosition(nextFront, VecUtil.fromWrongYaw(distance, yawFront), config.gauge.value());
        trackRear.getNextPosition(nextRear, VecUtil.fromWrongYaw(distance, yawRear), config.gauge.value());
        Vec3d nextFrontPos = nextFront.getUMCPos();
        Vec3d nextRearPos = nextRear.getUMCPos();

        /*
         * At a chunk boundary MovementTrack.findTrack can resolve to a different (or partially
         * loaded) rail than the one under the bogie.  getNextPosition then returns a point on
         * that other rail and deltaCenter below would drag the whole car there in one tick -
         * this is the "cars suddenly appear hundreds of blocks down the line" failure.  A bogie
         * may only ever travel about the requested distance along its own track, so reject a
         * destination that is wildly further away instead of applying it.
         */
        if (Simulation.workarounds && Math.abs(distance) > 0.0001) {
            double maxBogieTravel = Math.abs(distance) + MOVE_SLOP;
            if (!isFinite(nextFrontPos) || !isFinite(nextRearPos)
                    || positionFront.distanceToSquared(nextFrontPos) > maxBogieTravel * maxBogieTravel
                    || positionRear.distanceToSquared(nextRearPos) > maxBogieTravel * maxBogieTravel) {
                debugMove("REJECTED BOGUS BOGIE MOVE", this, distance,
                        positionFront + "->" + nextFrontPos + " " + positionRear + "->" + nextRearPos);
                return;
            }
        }

        if (!nextFrontPos.equals(positionFront) && !nextRearPos.equals(positionRear)) {
            yawFront = VecUtil.toWrongYaw(nextFrontPos.subtract(positionFront));
            yawRear = VecUtil.toWrongYaw(nextRearPos.subtract(positionRear));
            rollFront = (float) -nextFront.getRoll();
            rollRear = (float) -nextRear.getRoll();

            // TODO flatten this vector calculation
            Vec3d deltaCenter = nextFrontPos.subtract(position).scale(config.offsetRear)
                    .subtract(nextRearPos.subtract(position).scale(config.offsetFront))
                    .scale(-1/(config.offsetFront-config.offsetRear));

            Vec3d bogeyDelta = nextFrontPos.subtract(nextRearPos);
            yaw = VecUtil.toWrongYaw(bogeyDelta);
            roll = (float) Simulation.calculateRoll(rollFront, rollRear);
            pitch = (float) Math.toDegrees(FastMath.atan2(bogeyDelta.y, nextRearPos.distanceTo(nextFrontPos)));
            // TODO Rescale fixes issues with curves losing precision, but breaks when correcting stock positions
            position = position.add(deltaCenter/*.normalize().scale(distance)*/);
        }

        if (isReversed) {
            yawFront += 180;
            yawRear += 180;
            rollFront = -rollFront;
            rollRear = -rollRear;
            roll = - roll;
        }

        if (isTable) {
            yawFront = yaw;
            yawRear = yaw;
            rollFront = roll;
            rollRear = roll;
        }

        // Fix bogeys pointing in opposite directions
        if (DegreeFuncs.delta(yawFront, yaw) > 90 || DegreeFuncs.delta(yawFront, yawRear) > 90) {
            yawFront = yaw;
            yawRear = yaw;
            rollFront = roll;
            rollRear = roll;
        }
    }

    public double forcesNewtons() {
        double gradeForceNewtons = config.massKg * -9.8 * Math.sin(Math.toRadians(pitch)) * Config.ConfigBalance.slopeMultiplier;
        return config.tractiveEffortNewtons(Speed.fromMinecraft(velocity)) + gradeForceNewtons;
    }

    public boolean atRest() {
        return velocity == 0 && Math.abs(forcesNewtons()) < frictionNewtons();
    }

    public double frictionNewtons() {
        // https://evilgeniustech.com/idiotsGuideToRailroadPhysics/OtherLocomotiveForces/#rolling-resistance
        double rollingResistanceNewtons = config.rollingResistanceCoefficient * (config.massKg * 9.8);
        // https://www.arema.org/files/pubs/pgre/PGChapter2.pdf
        // ~15 lb/ton -> 0.01 weight ratio -> 0.001 uS with gravity
        double startingFriction = velocity == 0 ? 0.001 * config.massKg * 9.8 : 0;
        // TODO This is kinda directional?
        double blockResistanceNewtons = interferingResistance * 1000 * Config.ConfigDamage.blockHardness;

        double brakeAdhesionNewtons = config.designAdhesionNewtons * Math.min(1, Math.max(brakePressure, config.independentBrakePosition));

        this.sliding = false;
        if (brakeAdhesionNewtons > config.maximumAdhesionNewtons && Math.abs(velocity) > 0.01) {
            // WWWWWHHHEEEEE!!! SLIDING!!!!
            double kineticFriction = PhysicalMaterials.STEEL.kineticFriction(PhysicalMaterials.STEEL);
            brakeAdhesionNewtons = config.massKg * kineticFriction;
            this.sliding = true;
        }

        brakeAdhesionNewtons *= Config.ConfigBalance.brakeMultiplier;

        return rollingResistanceNewtons + blockResistanceNewtons + brakeAdhesionNewtons + directResistance + startingFriction;
    }

    private boolean checkTileType(TileRailBase base, TrackItems type) {
        return base != null
                && base.getParentTile() != null
                && base.getParentTile().info.settings.type == type;
    }
}
