package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.immersiverailroading.track.VecYPR;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.util.List;

public class TrackNode {
    public final TrackSingleGeometrySegment trackSingleGeometrySegment;
    public final int index;

    public final VecYPR direction;

    public TrackNode(TrackSingleGeometrySegment trackSingleGeometrySegment, int index, World world, Vec3i pos) {
        this.trackSingleGeometrySegment = trackSingleGeometrySegment;
        this.index = index;
        this.direction = trackSingleGeometrySegment.pointsCache.get(index);
    }

    public TrackNode(TrackSingleGeometrySegment trackSingleGeometrySegment, boolean isStart, World world, Vec3i pos) {
        this.trackSingleGeometrySegment = trackSingleGeometrySegment;
        List<VecYPR> points = trackSingleGeometrySegment.pointsCache;
        this.index = isStart ? 0 : points.size() - 1;
        this.direction = points.get(index);
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

//    public TrackNode getConn(TrackNode current, World world) {
//        //todo 思路：先根据方块坐标初步筛选segments，然后筛选点：先根据锥形范围筛选，距离阈值小于0.5 * gauge，再根据朝向夹角筛选，阈值小于90度，如果都符合的，优先选择夹角最优的
//
//
//    }
}
