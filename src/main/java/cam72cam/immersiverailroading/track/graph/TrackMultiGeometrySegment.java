package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.library.SwitchState;
import cam72cam.immersiverailroading.tile.TileRail;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrackMultiGeometrySegment {
    // todo 这些可有可无的冗余东西addition也许在之后拓扑层能缩短调用链简化代码微微提高效率？要么不做，要么至少要做到能溯源到上级方便获得上级的数据？
    //  不过考虑到加的东西很少，就先留着，等完成后如果确实没什么用还是删了
    private final long regionPos;// addition, not necessary
    private final Vec3i regionBlockPos;// addition, not necessary

    private final long buildTime;
    public final List<Map<Gauge, TrackSingleGeometrySegment>> paths;
    protected  int switchState;
    private int tableIndex;

    public Vec3i getBlockPos() {
        return TrackRegionUtil.toBlockPos(regionPos, regionBlockPos);
    }

    public TrackMultiGeometrySegment(TileRail trackBlock, World world) {
        this.regionPos = TrackRegionUtil.vecToRegion(trackBlock.getPos());
        this.regionBlockPos = TrackRegionUtil.toRegionBlockPos(trackBlock.getPos());

        buildTime = trackBlock.getBuildTimeMs();

        TrackSingleGeometrySegment segment = new TrackSingleGeometrySegment(trackBlock, world, 0, trackBlock.info.settings.gauge);
        paths = new ArrayList<>();
        Map<Gauge, TrackSingleGeometrySegment> branch = new HashMap<>();
        branch.put(trackBlock.info.settings.gauge, segment);
        paths.add(branch);

//        tableIndex = ;
        //todo: DO NOT use isSwitchForced()! it may cause crash!
//        switchState = trackBlock.isSwitchForced() ? trackBlock.info.switchForced.ordinal() : trackBlock.info.switchState.ordinal();
    }

    public void updateSwitchSate(SwitchState switchState) {
        this.switchState = switchState.ordinal();
    }

    public void updateTableState() {
        //todo
    }

    public TrackMultiGeometrySegment(ByteBuffer buffer, World world, long regionPos, Vec3i regionBlockPos) {
        this.regionPos = regionPos;
        this.regionBlockPos = regionBlockPos;

        int version = buffer.getInt(); // version
        if (version != 1) {
            throw new RuntimeException(String.format("Invalid multi track geometry segment data version %d", version));
        }

        buildTime = buffer.getLong(); // buildTime
        switchState = buffer.getInt(); // switch state
        tableIndex = buffer.getInt(); // table state

        paths = new ArrayList<>();
        int branchCount = buffer.getInt(); // branch count
        for(int i = 0; i < branchCount; i ++) {
            int segmentCount = buffer.getInt(); // segment count
            Map<Gauge, TrackSingleGeometrySegment> branch = new HashMap<>();
            for(int j = 0; j < segmentCount; j++) {
                Gauge gauge = Gauge.from(buffer.getDouble()); // segment gauge
                branch.put(gauge, new TrackSingleGeometrySegment(buffer, world, regionPos, regionBlockPos)); // single segment
            }
            paths.add(branch);
        }
    }

    public int sizeBytes() {
        int bytes = 0;

        bytes += Integer.BYTES; // version
        bytes += Long.BYTES; // buildTime
        bytes += Integer.BYTES; // switch state
        bytes += Integer.BYTES; // table state

        bytes += Integer.BYTES; // branch count
        for(int i = 0; i < paths.size(); i ++) {
            Map<Gauge, TrackSingleGeometrySegment> branch = paths.get(i);
            bytes += Integer.BYTES; // segment count
            for(Map.Entry<Gauge, TrackSingleGeometrySegment> entry : branch.entrySet()) {
                bytes += Double.BYTES; // segment gauge
                bytes += entry.getValue().sizeBytes(); // single segment
            }
        }

        return bytes;
    }

    public void write(ByteBuffer buffer) {
        buffer.putInt(1); // version
        buffer.putLong(buildTime); // buildTime
        buffer.putInt(switchState); // switch state
        buffer.putInt(tableIndex); // table state

        buffer.putInt(paths.size()); // branch count
        for(int i = 0; i < paths.size(); i ++) {
            Map<Gauge, TrackSingleGeometrySegment> branch = paths.get(i);
            buffer.putInt(branch.size()); // segment count
            for(Map.Entry<Gauge, TrackSingleGeometrySegment> entry : branch.entrySet()) {
                buffer.putDouble(entry.getKey().value()); // segment gauge
                entry.getValue().write(buffer); // single segment
            }
        }
    }

    public List<Vec3i> getPositionsCache() {
        List<Vec3i> positionCache = new ArrayList<>();
        for(Map<Gauge, TrackSingleGeometrySegment> branch : paths) {
            for(Map.Entry<Gauge, TrackSingleGeometrySegment> entry : branch.entrySet()) {
                positionCache.addAll(entry.getValue().positionsCache);
            }
        }
        return positionCache;
    }
}
