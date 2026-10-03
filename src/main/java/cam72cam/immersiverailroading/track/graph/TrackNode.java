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
        double minAngle = 90;
        double minDist = 0x3f;

        // 3D 前向：由 yaw + pitch 决定
        // MC 约定 yaw=0 -> +Z, yaw=90 -> -X, pitch>0 朝下
        double yawRad = Math.toRadians(this.point.getYaw());
        double pitchRad = Math.toRadians(this.point.getPitch());
        double cp = Math.cos(pitchRad);
        Vec3d forwardVec = new Vec3d(
                -Math.sin(yawRad) * cp,
                -Math.sin(pitchRad),
                Math.cos(yawRad) * cp
        );
        if (!forward) {
            forwardVec = forwardVec.scale(-1);
        }
        final double cosCone = Math.cos(Math.toRadians(45)); // 锥形半角 45°

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
                            for (int i = 0; i < points.size(); i++) {
                                VecYPR cand = points.get(i)
                                        .add(new Vec3d(parentPos))
                                        .add(single.getValue()
                                                .getAndUpdateBuilder(world, parentPos).info.placementInfo.placementPosition);

                                // 距离过滤
                                double d = cand.distanceTo(this.point);
                                if (d >= single.getValue().gauge.value() * 0.5) continue;

                                // 3D 锥形过滤
                                Vec3d offset = new Vec3d(
                                        cand.x - this.point.x,
                                        cand.y - this.point.y,
                                        cand.z - this.point.z
                                );
                                double dist = offset.length();
                                if (dist > 1e-4) {
                                    Vec3d dir = offset.scale(1.0 / dist);
                                    if (dir.dotProduct(forwardVec) < cosCone) continue;
                                }

                                double angle = angleBetween(cand.toMatrix3(), this.point.toMatrix3());
                                if (angle < minAngle && d < minDist) {
                                    minAngle = angle;
                                    minDist = d;
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

    public static double angleBetween(Matrix3 a, Matrix3 b) {
        // 相对旋转 = a^-1 * b = a^T * b （对纯旋转矩阵）
        Quaternion qa = a.toQuaternion();
        Quaternion qb = b.toQuaternion();

        // 点积
        double dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
        // 取绝对值：q 和 -q 表示同一旋转
        double absDot = Math.min(1.0, Math.abs(dot));
        return 2.0 * Math.acos(absDot);
    }
}
