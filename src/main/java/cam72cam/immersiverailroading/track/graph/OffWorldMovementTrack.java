package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.Config;
import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.thirdparty.trackapi.IRPathingData;
import cam72cam.immersiverailroading.track.VecYPR;
import cam72cam.immersiverailroading.util.RailInfo;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.util.List;
import java.util.Map;

public class OffWorldMovementTrack {
    private static final int MAX_ITERATIONS = 64;

    public static void getNextPosition(IRPathingData current, Vec3d motion, double gauge, World world) {
        WorldData data = WorldData.get(world);
        if (data == null) return;

        double motionDist = motion.length();
        if (motionDist < 1e-6) return;

        if (current.getTopoEdge() == null) {
            if (!initCurrent(current, world, motion, gauge)) return;
        }

        double[] posOut = new double[4];

        TrackEdge cachedEdge = null;
        TrackNode cachedFrom = null;
        TrackNode cachedTo = null;
        double cachedEdgeDist = 0;
        Vec3d cachedOffset = null;

        for (int iter = 0; iter < MAX_ITERATIONS && motionDist > 1e-6; iter++) {
            TrackEdge edge = current.getTopoEdge();
            if (edge == null) return;

            if (!edge.equals(cachedEdge)) {
                TrackNode from = data.nodes.get(edge.from);
                TrackNode to = data.nodes.get(edge.to);
                if (from == null || to == null) return;
                cachedEdge = edge;
                cachedFrom = from;
                cachedTo = to;
                cachedEdgeDist = edgeDistance(edge, from, to);
                cachedOffset = singleOffset(from, world);
            }

            TrackNode from = cachedFrom;
            TrackNode to = cachedTo;
            double edgeDist = cachedEdgeDist;
            Vec3d offset = cachedOffset;
            if (edgeDist < 1e-9) return;

            double t = current.getTopoT();
            Vec3d localDir = localDirectionAt(edge, from, to, t);
            double dot = localDir.x * motion.x + localDir.y * motion.y + localDir.z * motion.z;
            boolean towardTo = dot > 0;

            double rollMult;
            if (edge.edgeType == TrackEdge.EdgeType.GAP) {
                rollMult = 1.0;
            } else {
                int idxDelta = to.index - from.index;
                boolean alongIncrement = (towardTo == (idxDelta > 0));
                rollMult = alongIncrement ? 1.0 : -1.0;
            }

            double tTarget = towardTo ? 1.0 : 0.0;
            double distToEnd = edgeDist * Math.abs(tTarget - t);

            // System.out.println("[GNP LOOP] iter=" + iter
            //         + " edge=" + edgeToString(edge)
            //         + " t=" + t
            //         + " edgeDist=" + edgeDist
            //         + " towardTo=" + towardTo
            //         + " tTarget=" + tTarget
            //         + " distToEnd=" + distToEnd
            //         + " motionDist=" + motionDist
            //         + " localDir=" + localDir
            //         + " motion=" + motion);

            if (motionDist < distToEnd) {
                double tDelta = motionDist / edgeDist;
                double newT = towardTo ? t + tDelta : t - tDelta;
                positionAt(edge, from, to, newT, offset, posOut, world);
                double finalRoll = posOut[3] * rollMult;
                current.advanceTo(edge, newT, finalRoll);
                current.advanceTo(new Vec3d(posOut[0], posOut[1], posOut[2]), finalRoll);
                return;
            }

            motionDist -= distToEnd;
            positionAt(edge, from, to, tTarget, offset, posOut, world);
            double endRoll = posOut[3] * rollMult;
            current.advanceTo(new Vec3d(posOut[0], posOut[1], posOut[2]), endRoll);

            TrackNodeId endNodeId = towardTo ? edge.to : edge.from;
            TrackEdge nextEdge = pickNextEdge(data, edge, endNodeId, motion);
            if (nextEdge == null) {
                current.advanceTo(edge, tTarget, endRoll);
                return;
            }

            if (!data.nodes.containsKey(nextEdge.from) || !data.nodes.containsKey(nextEdge.to)) {
                current.advanceTo(edge, tTarget, endRoll);
                return;
            }

            // System.out.println("[GNP PICK] iter=" + iter
            //         + " endNodeId=" + idToString(endNodeId)
            //         + " nextEdge=" + edgeToString(nextEdge));

            double startT = nextEdge.from.equals(endNodeId) ? 0.0 : 1.0;

            TrackNode nFrom = data.nodes.get(nextEdge.from);
            TrackNode nTo = data.nodes.get(nextEdge.to);
            Vec3d nOffset = singleOffset(nFrom, world);
            positionAt(nextEdge, nFrom, nTo, startT, nOffset, posOut, world);
            double startRoll = posOut[3];

            current.advanceTo(nextEdge, startT, startRoll);
            current.advanceTo(new Vec3d(posOut[0], posOut[1], posOut[2]), startRoll);
        }

        // System.out.println("[GNP EXIT] iter limit reached, finalT=" + current.getTopoT()
        //         + " edge=" + current.getTopoEdge()
        //         + " motionDistLeft=" + motionDist);
    }

    private static String idToString(TrackNodeId id) {
        if (id == null) return "null";
        int rx = TrackRegionUtil.regionX(id.regionPos);
        int rz = TrackRegionUtil.regionZ(id.regionPos);
        return "[" + rx + "," + rz + "]"
                + id.regionBlockPos
                + "/p" + id.pathIndex
                + "/g" + id.gauge
                + "/i" + id.pointIndex;
    }

    private static String edgeToString(TrackEdge e) {
        if (e == null) return "null";
        return e.edgeType + "{" + idToString(e.from) + "->" + idToString(e.to) + "}";
    }

    private static double edgeDistance(TrackEdge edge, TrackNode from, TrackNode to) {
        if (edge.edgeType == TrackEdge.EdgeType.GAP) {
            double dx = to.point.x - from.point.x;
            double dy = to.point.y - from.point.y;
            double dz = to.point.z - from.point.z;
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        List<VecYPR> points = from.trackSingleGeometrySegment.pointsCache;
        double d = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            VecYPR a = points.get(i);
            VecYPR b = points.get(i + 1);
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double dz = b.z - a.z;
            d += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        return d;
    }

    private static void positionAt(TrackEdge edge, TrackNode from, TrackNode to, double t, Vec3d offset, double[] out, World world) {
        if (edge.edgeType == TrackEdge.EdgeType.GAP) {
            double invT = 1.0 - t;
            double hFrom = from.trackSingleGeometrySegment.getAndUpdateBuilder(world).info.getTrackHeight()
                    * from.trackSingleGeometrySegment.gauge.scale();
            double hTo = to.trackSingleGeometrySegment.getAndUpdateBuilder(world).info.getTrackHeight()
                    * to.trackSingleGeometrySegment.gauge.scale();
            double h = hFrom * invT + hTo * t;

            double fromRollSign = (from.index == 0) ? -1.0 : 1.0;
            double toRollSign = (to.index == 0) ? 1.0 : -1.0;
            double fromRoll = from.point.getRoll() * fromRollSign;
            double toRoll = to.point.getRoll() * toRollSign;

            out[0] = from.point.x * invT + to.point.x * t;
            out[1] = from.point.y * invT + to.point.y * t + h;
            out[2] = from.point.z * invT + to.point.z * t;
            out[3] = fromRoll * invT + toRoll * t;
            return;
        }
        List<VecYPR> points = from.trackSingleGeometrySegment.pointsCache;
        int lastIdx = points.size() - 1;
        double actualIdx = from.index + (to.index - from.index) * t;
        int i = (int) Math.floor(actualIdx);
        double frac = actualIdx - i;
        if (i < 0) { i = 0; frac = 0; }
        if (i >= lastIdx) { i = lastIdx; frac = 0; }

        VecYPR a = points.get(i);
        if (frac < 1e-9 || i >= lastIdx) {
            out[0] = a.x + offset.x;
            out[1] = a.y + offset.y;
            out[2] = a.z + offset.z;
            out[3] = a.getRoll();
            return;
        }
        VecYPR b = points.get(i + 1);
        out[0] = a.x + (b.x - a.x) * frac + offset.x;
        out[1] = a.y + (b.y - a.y) * frac + offset.y;
        out[2] = a.z + (b.z - a.z) * frac + offset.z;
        out[3] = a.getRoll() + (b.getRoll() - a.getRoll()) * frac;
    }

    private static Vec3d localDirectionAt(TrackEdge edge, TrackNode from, TrackNode to, double t) {
        if (edge.edgeType == TrackEdge.EdgeType.GAP) {
            double dx = to.point.x - from.point.x;
            double dy = to.point.y - from.point.y;
            double dz = to.point.z - from.point.z;
            double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (len < 1e-9) return new Vec3d(0, 0, 1);
            return new Vec3d(dx / len, dy / len, dz / len);
        }
        List<VecYPR> points = from.trackSingleGeometrySegment.pointsCache;
        int lastIdx = points.size() - 1;
        double actualIdx = from.index + (to.index - from.index) * t;
        int i = (int) Math.floor(actualIdx);
        if (i < 0) i = 0;
        if (i >= lastIdx) i = lastIdx - 1;
        VecYPR a = points.get(i);
        VecYPR b = points.get(i + 1);
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dz = b.z - a.z;
        double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-9) return new Vec3d(0, 0, 1);
        double sign = (from.index < to.index) ? 1.0 : -1.0;
        return new Vec3d(sign * dx / len, sign * dy / len, sign * dz / len);
    }

    private static Vec3d singleOffset(TrackNode node, World world) {
        TrackSingleGeometrySegment single = node.trackSingleGeometrySegment;
        Vec3i parentPos = TrackRegionUtil.toBlockPos(node.id.regionPos, node.id.regionBlockPos);
        RailInfo info = single.getAndUpdateBuilder(world).info;
        Vec3d pp = info.placementInfo.placementPosition;
        double h = info.getTrackHeight() * single.gauge.scale();
        return new Vec3d(parentPos.x + pp.x, parentPos.y + pp.y + h, parentPos.z + pp.z);
    }

    private static Vec3d singleOffsetStatic(TrackSingleGeometrySegment single, Vec3i absPos, World world) {
        RailInfo info = single.getAndUpdateBuilder(world).info;
        Vec3d pp = info.placementInfo.placementPosition;
        double h = info.getTrackHeight() * single.gauge.scale();
        return new Vec3d(absPos.x + pp.x, absPos.y + pp.y + h, absPos.z + pp.z);
    }

    private static TrackEdge pickNextEdge(WorldData data, TrackEdge incoming, TrackNodeId endNodeId, Vec3d motion) {
        List<TrackEdge> candidates = data.outgoing.get(endNodeId);
        if (candidates == null || candidates.isEmpty()) return null;

        double motionLen = motion.length();
        if (motionLen < 1e-9) {
            for (TrackEdge e : candidates) {
                if (e.equals(incoming)) continue;
                if (e.from.equals(incoming.to) && e.to.equals(incoming.from)) continue;
                return e;
            }
            return null;
        }

        TrackEdge best = null;
        double bestCos = -Double.MAX_VALUE;

        for (TrackEdge e : candidates) {
            if (e.equals(incoming)) continue;
            if (e.from.equals(incoming.to) && e.to.equals(incoming.from)) continue;

            TrackNode eFrom = data.nodes.get(e.from);
            TrackNode eTo = data.nodes.get(e.to);
            if (eFrom == null || eTo == null) continue;

            double dx = eTo.point.x - eFrom.point.x;
            double dy = eTo.point.y - eFrom.point.y;
            double dz = eTo.point.z - eFrom.point.z;
            double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
            double cos;
            if (len < 1e-9) {
                cos = 0.0;
            } else {
                cos = (dx * motion.x + dy * motion.y + dz * motion.z) / (len * motionLen);
            }
            if (cos > bestCos) {
                bestCos = cos;
                best = e;
            }
        }

        return best;
    }

    public static boolean initCurrent(IRPathingData current, World world, Vec3d motion, double gauge) {
        WorldData data = WorldData.get(world);
        if (data == null) return false;

        Vec3d pos = current.getTopoPos();

        double maxRadius = Math.max(gauge * 2, 2);
        int hori = (int) Math.ceil(maxRadius);
        int vert = 1;

        TrackSingleGeometrySegment bestSingle = null;
        Vec3i bestAbsPos = null;
        int bestPathIndex = 0;
        Gauge bestGauge = null;
        double bestLocalIndex = 0;
        double bestDistSq = Double.MAX_VALUE;

        for (int x = -hori; x <= hori; x++) {
            for (int y = -vert; y <= vert; y++) {
                for (int z = -hori; z <= hori; z++) {
                    Vec3i scan = new Vec3i(pos).add(x, y, z);
                    TrackRegion region = data.getRegion(scan, false);
                    if (region == null) continue;
                    List<Vec3i> parents = region.trackBlockParents.get(TrackRegionUtil.toRegionBlockPos(scan));
                    if (parents == null) continue;
                    for (Vec3i parentRelPos : parents) {
                        Vec3i absPos = region.toBlockPos(parentRelPos);
                        TrackMultiGeometrySegment block = data.getTrackBlock(absPos);
                        if (block == null) continue;

                        for (int pathIndex = 0; pathIndex < block.paths.size(); pathIndex++) {
                            for (Map.Entry<Gauge, TrackSingleGeometrySegment> e : block.paths.get(pathIndex).entrySet()) {
                                TrackSingleGeometrySegment single = e.getValue();
                                List<VecYPR> points = single.pointsCache;
                                if (points.isEmpty()) continue;

                                Vec3d offset = singleOffsetStatic(single, absPos, world);

                                for (int i = 0; i < points.size(); i++) {
                                    VecYPR pv = points.get(i);
                                    double px = pv.x + offset.x;
                                    double py = pv.y + offset.y;
                                    double pz = pv.z + offset.z;
                                    double dx0 = pos.x - px, dy0 = pos.y - py, dz0 = pos.z - pz;
                                    double d2 = dx0 * dx0 + dy0 * dy0 + dz0 * dz0;
                                    if (d2 < bestDistSq) {
                                        bestDistSq = d2;
                                        bestSingle = single;
                                        bestAbsPos = absPos;
                                        bestPathIndex = pathIndex;
                                        bestGauge = e.getKey();
                                        bestLocalIndex = i;
                                    }
                                    if (i < points.size() - 1) {
                                        VecYPR pv2 = points.get(i + 1);
                                        double ax = pv2.x + offset.x - px;
                                        double ay = pv2.y + offset.y - py;
                                        double az = pv2.z + offset.z - pz;
                                        double segLen2 = ax * ax + ay * ay + az * az;
                                        if (segLen2 > 1e-9) {
                                            double tt = (dx0 * ax + dy0 * ay + dz0 * az) / segLen2;
                                            if (tt < 0) tt = 0;
                                            if (tt > 1) tt = 1;
                                            double cx = px + ax * tt - pos.x;
                                            double cy = py + ay * tt - pos.y;
                                            double cz = pz + az * tt - pos.z;
                                            double d2b = cx * cx + cy * cy + cz * cz;
                                            if (d2b < bestDistSq) {
                                                bestDistSq = d2b;
                                                bestSingle = single;
                                                bestAbsPos = absPos;
                                                bestPathIndex = pathIndex;
                                                bestGauge = e.getKey();
                                                bestLocalIndex = i + tt;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (bestSingle == null || bestDistSq > maxRadius * maxRadius) return false;

        int lastIdx = bestSingle.pointsCache.size() - 1;
        if (lastIdx <= 0) return false;

        long regionId = TrackRegionUtil.vecToRegion(bestAbsPos);
        Vec3i relBlockPos = TrackRegionUtil.toRegionBlockPos(bestAbsPos);

        TrackNodeId startId = new TrackNodeId(regionId, relBlockPos, bestPathIndex, bestGauge, 0);
        TrackNodeId endId = new TrackNodeId(regionId, relBlockPos, bestPathIndex, bestGauge, lastIdx);

        TrackNode startNode = data.nodes.get(startId);
        TrackNode endNode = data.nodes.get(endId);
        if (startNode == null || endNode == null) return false;

        double t = bestLocalIndex / lastIdx;

        TrackEdge forwardEdge = null;
        TrackEdge backwardEdge = null;
        for (TrackEdge e : data.edges) {
            if (e.edgeType != TrackEdge.EdgeType.NORMAL) continue;
            if (e.from.equals(startId) && e.to.equals(endId)) forwardEdge = e;
            if (e.from.equals(endId) && e.to.equals(startId)) backwardEdge = e;
        }
        if (forwardEdge == null) return false;

        Vec3d localDir = localDirectionAt(forwardEdge, startNode, endNode, t);
        boolean useForward = localDir.dotProduct(motion) >= 0;

        TrackEdge chosen = useForward ? forwardEdge : backwardEdge;
        if (chosen == null) return false;

        double chosenT = useForward ? t : 1.0 - t;
        double rollMult = useForward ? 1.0 : -1.0;

        TrackNode chosenFrom = data.nodes.get(chosen.from);
        TrackNode chosenTo = data.nodes.get(chosen.to);
        if (chosenFrom == null || chosenTo == null) return false;

        Vec3d offset = singleOffset(chosenFrom, world);
        double[] posOut = new double[4];
        positionAt(chosen, chosenFrom, chosenTo, chosenT, offset, posOut, world);
        double initRoll = posOut[3] * rollMult;
        current.advanceTo(chosen, chosenT, initRoll);
        current.advanceTo(new Vec3d(posOut[0], posOut[1], posOut[2]), initRoll);
        return true;
    }

    public static double projectOntoEdge(TrackEdge edge, Vec3d pos, World world) {
        WorldData data = WorldData.get(world);
        if (data == null) return -1;
        TrackNode from = data.nodes.get(edge.from);
        TrackNode to = data.nodes.get(edge.to);
        if (from == null || to == null) return -1;

        double maxDist = Math.max(1.0, from.trackSingleGeometrySegment.gauge.value() * 0.5);
        double maxDistSq = maxDist * maxDist;

        if (edge.edgeType == TrackEdge.EdgeType.GAP) {
            double hFrom = from.trackSingleGeometrySegment.getAndUpdateBuilder(world).info.getTrackHeight()
                    * from.trackSingleGeometrySegment.gauge.scale();
            double hTo = to.trackSingleGeometrySegment.getAndUpdateBuilder(world).info.getTrackHeight()
                    * to.trackSingleGeometrySegment.gauge.scale();
            double ay = from.point.y + hFrom;
            double by = to.point.y + hTo;
            double ax = to.point.x - from.point.x;
            double ayy = by - ay;
            double az = to.point.z - from.point.z;
            double abLen2 = ax * ax + ayy * ayy + az * az;
            if (abLen2 < 1e-9) return -1;
            double t = ((pos.x - from.point.x) * ax + (pos.y - ay) * ayy + (pos.z - from.point.z) * az) / abLen2;
            t = Math.max(0, Math.min(1, t));
            double cx = from.point.x + ax * t - pos.x;
            double cy = ay + ayy * t - pos.y;
            double cz = from.point.z + az * t - pos.z;
            if (cx * cx + cy * cy + cz * cz > maxDistSq) return -1;
            return t;
        }

        List<VecYPR> points = from.trackSingleGeometrySegment.pointsCache;
        int lastIdx = points.size() - 1;
        if (lastIdx < 0) return -1;
        Vec3d offset = singleOffset(from, world);
        int denom = to.index - from.index;
        if (denom == 0) return -1;

        double bestDistSq = Double.MAX_VALUE;
        double bestT = -1;

        int lo = Math.min(from.index, to.index);
        int hi = Math.max(from.index, to.index);
        lo = Math.max(0, lo);
        hi = Math.min(lastIdx, hi);

        for (int i = lo; i < hi; i++) {
            VecYPR pa = points.get(i);
            VecYPR pb = points.get(i + 1);
            double ax = pa.x + offset.x, ay = pa.y + offset.y, az = pa.z + offset.z;
            double bx = pb.x + offset.x, by = pb.y + offset.y, bz = pb.z + offset.z;
            double dx = bx - ax, dy = by - ay, dz = bz - az;
            double segLen2 = dx * dx + dy * dy + dz * dz;
            if (segLen2 < 1e-9) continue;

            double tt = ((pos.x - ax) * dx + (pos.y - ay) * dy + (pos.z - az) * dz) / segLen2;
            tt = Math.max(0, Math.min(1, tt));
            double cx = ax + dx * tt - pos.x;
            double cy = ay + dy * tt - pos.y;
            double cz = az + dz * tt - pos.z;
            double d2 = cx * cx + cy * cy + cz * cz;
            if (d2 >= bestDistSq) continue;

            double idx = i + tt;
            double t = (idx - from.index) / (double) denom;
            if (t < 0 || t > 1) continue;

            bestDistSq = d2;
            bestT = t;
        }

        if (bestT < 0 || bestDistSq > maxDistSq) return -1;
        return bestT;
    }

    public static Vec3d positionOnEdge(TrackEdge edge, double t, World world) {
        WorldData data = WorldData.get(world);
        if (data == null) return Vec3d.ZERO;
        TrackNode from = data.nodes.get(edge.from);
        TrackNode to = data.nodes.get(edge.to);
        if (from == null || to == null) return Vec3d.ZERO;
        Vec3d offset = singleOffset(from, world);
        double[] posOut = new double[4];
        positionAt(edge, from, to, t, offset, posOut, world);
        return new Vec3d(posOut[0], posOut[1], posOut[2]);
    }
}