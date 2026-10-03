package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.track.VecYPR;
import cam72cam.mod.math.Matrix3;
import cam72cam.mod.math.Quaternion;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.util.List;
import java.util.Map;

public class TrackNode {
    public final TrackSingleGeometrySegment trackSingleGeometrySegment;
    public final int index;

    public final VecYPR point;

    public TrackNode(TrackSingleGeometrySegment trackSingleGeometrySegment, int index, World world, Vec3i pos) {
        this.trackSingleGeometrySegment = trackSingleGeometrySegment;
        this.index = index;
        this.point = trackSingleGeometrySegment.pointsCache.get(index).add(new Vec3d(pos)).add(trackSingleGeometrySegment.getAndUpdateBuilder(world, pos).info.placementInfo.placementPosition);
    }

    public TrackNode(TrackSingleGeometrySegment trackSingleGeometrySegment, boolean isStart, World world, Vec3i pos) {
        this.trackSingleGeometrySegment = trackSingleGeometrySegment;
        List<VecYPR> points = trackSingleGeometrySegment.pointsCache;
        this.index = isStart ? 0 : points.size() - 1;
        this.point = points.get(index).add(new Vec3d(pos)).add(trackSingleGeometrySegment.getAndUpdateBuilder(world, pos).info.placementInfo.placementPosition);
    }

    public TrackNode offset(int indexOffset, World world, Vec3i pos) {
        List<VecYPR> points = trackSingleGeometrySegment.pointsCache;
        int newIndex = index + indexOffset;
        if(newIndex >= 0 && newIndex < points.size()) {
            return new TrackNode(trackSingleGeometrySegment, newIndex, world, pos);
        } else {
            ImmersiveRailroading.error("invalid trackSingleGeometrySegment index %s: value out of boundary (%s)", newIndex, points.size());
            return this;
        }
    }

    public TrackNode getConn(World world, Vec3i current, boolean forward) {
        int hori = Math.max((int) (trackSingleGeometrySegment.gauge.scale() * 2), 1);
        int vert = 1;
        TrackNode best = null;
        double bestScore = Double.MAX_VALUE;

        final double maxAngleRad = Math.toRadians(20);
        final double cosCone = Math.cos(maxAngleRad);

        // 当前点的 3D 前向（yaw + pitch，不含 roll）
        Vec3d forwardVec = forwardVector(this.point);
        // 位置锥形方向：forward 决定往前还是往后扫
        Vec3d coneVec = forward ? forwardVec : forwardVec.scale(-1);

        for (int x = -hori; x <= hori; x++) {
            for (int y = -vert; y <= vert; y++) {
                for (int z = -hori; z <= hori; z++) {
                    Vec3i scan = new Vec3i(this.point).add(x, y, z);
                    TrackRegion trackRegion = WorldData.get(world).getRegion(scan, false);
                    if (trackRegion == null) continue;
                    List<Vec3i> parents = trackRegion.trackBlockParents.get(TrackRegion.toRegionBlockPos(scan));
                    if (parents == null) continue;
                    for (Vec3i parentRelPos : parents) {
                        long regionId = WorldData.vecToRegion(scan);
                        Vec3i parentPos = TrackRegion.toBlockPos(regionId, parentRelPos);
                        if (parentPos.equals(current)) continue;

                        TrackMultiGeometrySegment trackBlock = WorldData.get(world).getTrackBlock(parentPos);
                        if (trackBlock == null) continue;
                        for (Map.Entry<Gauge, TrackSingleGeometrySegment> single : trackBlock.paths.getFirst().entrySet()) {
                            List<VecYPR> points = single.getValue().pointsCache;
                            double maxDist = single.getValue().gauge.value();
                            for (int i = 0; i < points.size(); i++) {
                                VecYPR cand = points.get(i)
                                        .add(new Vec3d(parentPos))
                                        .add(single.getValue()
                                                .getAndUpdateBuilder(world, parentPos).info.placementInfo.placementPosition);

                                // 距离硬上限
                                double d = cand.distanceTo(this.point);
                                if (d >= maxDist) continue;

                                // 几乎重合：直接认定
                                if (d < 0.1) {
                                    return new TrackNode(single.getValue(), i, world, parentPos);
                                }

                                // 3D 位置锥形
                                Vec3d offset = new Vec3d(
                                        cand.x - this.point.x,
                                        cand.y - this.point.y,
                                        cand.z - this.point.z
                                );
                                double dist = offset.length();
                                if (dist > 1e-4) {
                                    Vec3d dir = offset.scale(1.0 / dist);
                                    if (dir.dotProduct(coneVec) < cosCone) continue;
                                }

                                // 姿态前向夹角（不含 roll）
                                Vec3d candForward = forwardVector(cand);
                                double dotFF = forwardVec.dotProduct(candForward);
                                double absDot = Math.max(0.0, Math.min(1.0, Math.abs(dotFF)));
                                double angleRad = Math.acos(absDot); // [0, π/2]
                                if (angleRad > maxAngleRad) continue;

                                // 组合评分：距离和角度各自归一化后加权
                                double distNorm  = d / maxDist;
                                double angleNorm = angleRad / maxAngleRad;
                                double score = 0.7 * distNorm + 0.3 * angleNorm;

                                if (score < bestScore) {
                                    bestScore = score;
                                    best = new TrackNode(single.getValue(), i, world, parentPos);
                                }
                            }
                        }
                    }
                }
            }
        }
        return best;
    }

    // 3D 前向：由 yaw + pitch 决定，roll 不参与
    // 3D 前向：VecYPR 的 yaw 是 IR 约定，先转成 MC yaw；pitch 不参与 roll
    private static Vec3d forwardVector(VecYPR p) {
        // IR yaw -> MC yaw（和 TrackSnapUtil 里的 yawHead 转换一致）
        float mcYaw = ((540 - p.getYaw()) % 360 + 180) % 360;
        double yawRad = Math.toRadians(mcYaw);
        double pitchRad = Math.toRadians(p.getPitch());
        double cp = Math.cos(pitchRad);
        return new Vec3d(
                -Math.sin(yawRad) * cp,
                -Math.sin(pitchRad),
                Math.cos(yawRad) * cp
        );
    }
}
