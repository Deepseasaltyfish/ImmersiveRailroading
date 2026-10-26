package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.track.VecYPR;

import java.util.HashSet;
import java.util.Set;

// todo 暂时采用单向边，如果有需要可以考虑单边加方向标记
public class TrackEdge {
    public final TrackNodeId from;
    public final TrackNodeId to;
    public final EdgeType edgeType;
    public final Set<Long> coveringRegions = new HashSet<>();

    /** TABLE_GAP 专用：应用 table 变换后的端点姿态。null 时用 TrackNode.point */
    public final VecYPR customFrom;
    public final VecYPR customTo;

    public TrackEdge(TrackNodeId from, TrackNodeId to, EdgeType edgeType) {
        this(from, to, edgeType, null, null);
    }

    public TrackEdge(TrackNodeId from, TrackNodeId to, EdgeType edgeType, VecYPR customFrom, VecYPR customTo) {
        this.from = from;
        this.to = to;
        this.edgeType = edgeType;
        this.customFrom = customFrom;
        this.customTo = customTo;
    }

    public enum EdgeType {
        NORMAL,     // single内部相邻点
        GAP,        // 连接2个single的节点
        TABLE_GAP   // table（转盘/移车台）与外部轨道的动态连接
    }

    @Override
    public int hashCode() {
        int h = from.hashCode();
        h = 31 * h + to.hashCode();
        h = 31 * h + edgeType.hashCode();
        return h;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof TrackEdge)) return false;
        TrackEdge o = (TrackEdge) obj;
        return from.equals(o.from) && to.equals(o.to) && edgeType == o.edgeType;
    }
}