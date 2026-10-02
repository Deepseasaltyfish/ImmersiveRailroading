package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.render.util.Color;
import cam72cam.immersiverailroading.track.VecYPR;
import cam72cam.immersiverailroading.util.RailInfo;
import cam72cam.mod.math.Matrix3;
import cam72cam.mod.math.Quaternion;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.render.opengl.RenderState;
import cam72cam.mod.world.World;

import java.util.List;
import java.util.Map;

public class TrackNode {
    public final TrackSingleGeometrySegment trackSingleGeometrySegment;
    public final int index;

    public final VecYPR direction;

    public TrackNode(TrackSingleGeometrySegment trackSingleGeometrySegment, int index, World world, Vec3i pos) {
        this.trackSingleGeometrySegment = trackSingleGeometrySegment;
        this.index = index;
        this.direction = trackSingleGeometrySegment.pointsCache.get(index).add(new Vec3d(pos));
    }

    public TrackNode(TrackSingleGeometrySegment trackSingleGeometrySegment, boolean isStart, World world, Vec3i pos) {
        this.trackSingleGeometrySegment = trackSingleGeometrySegment;
        List<VecYPR> points = trackSingleGeometrySegment.pointsCache;
        this.index = isStart ? 0 : points.size() - 1;
        this.direction = points.get(index).add(new Vec3d(pos));
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

    public TrackNode getConn(World world) {
        //todo 思路：先根据方块坐标初步筛选segments，然后筛选点：先根据前向锥形范围筛选，距离阈值小于0.5 * gauge，夹角筛选阈值小于45度，如果都符合的，优先选择夹角最优的
        int hori = Math.max((int) (trackSingleGeometrySegment.gauge.scale() * 2), 1);
        int vert = 1;
        TrackNode best = null;
        double minAngle = 90;
        for (int x = -hori; x <= hori; x++) {
            for (int y = -vert; y <= vert; y++) {
                for (int z = -hori; z <= hori; z++) {
                    Vec3i scan = new Vec3i(direction).add(x, y, z);
                    TrackRegion trackRegion = WorldData.get(world).getRegion(scan, false);
                    if(trackRegion == null) continue;
                    List<Vec3i> parents = trackRegion.trackBlockParents.get(scan);
                    for(Vec3i parentPos : parents) {
                        TrackMultiGeometrySegment trackBlock = WorldData.get(world).getTrackBlock(parentPos);
                        for(Map.Entry<Gauge, TrackSingleGeometrySegment> single : trackBlock.paths.getFirst().entrySet()){
                            List<VecYPR> points = single.getValue().pointsCache;
                            for(int i = 0; i < points.size(); i ++) {
                                VecYPR point = points.get(i);
                                point = point.add(new Vec3d(parentPos));
                                if(point.distanceTo(direction) < single.getValue().gauge.value() * 0.5) {
                                    double angle = angleBetween(point.toMatrix3(), direction.toMatrix3());
                                    if(angle < minAngle) {
                                        minAngle = angle;
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
