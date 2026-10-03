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
        for (int x = -hori; x <= hori; x++) {
            for (int y = -vert; y <= vert; y++) {
                for (int z = -hori; z <= hori; z++) {
                    Vec3i scan = new Vec3i(point).add(x, y, z);
                    TrackRegion trackRegion = WorldData.get(world).getRegion(scan, false);
                    if(trackRegion == null) continue;
                    List<Vec3i> parents = trackRegion.trackBlockParents.get(TrackRegion.toRegionBlockPos(scan));
                    if(parents == null) continue;
                    for(Vec3i parentRelPos : parents) {
                        long regionId = WorldData.vecToRegion(scan);
                        Vec3i parentPos = TrackRegion.toBlockPos(regionId, parentRelPos);
                        if(parentPos.equals(current)) continue;

                        TrackMultiGeometrySegment trackBlock = WorldData.get(world).getTrackBlock(parentPos);
                        for(Map.Entry<Gauge, TrackSingleGeometrySegment> single : trackBlock.paths.getFirst().entrySet()){
                            List<VecYPR> points = single.getValue().pointsCache;
                            for(int i = 0; i < points.size(); i ++) {
                                VecYPR point = points.get(i);
                                point = point.add(new Vec3d(parentPos)).add(single.getValue().getAndUpdateBuilder(world, parentPos).info.placementInfo.placementPosition);
                                if(point.distanceTo(this.point) < single.getValue().gauge.value() * 0.5) {
                                    double angle = angleBetween(point.toMatrix3(), this.point.toMatrix3());
                                    if(angle < minAngle && point.distanceTo(this.point) < minDist) {
                                        minAngle = angle;
                                        minDist = point.distanceTo(this.point);
                                        best = new TrackNode(single.getValue(), i, world, parentPos);
                                    }
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
