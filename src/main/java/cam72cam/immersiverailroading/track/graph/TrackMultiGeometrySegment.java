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
    private final long buildTime;
    private final List<Map<Gauge, TrackSingleGeometrySegment>> paths;
    private int switchState;
    private int tableIndex;

    public TrackMultiGeometrySegment(TileRail trackBlock, World world, Vec3i pos) {
        buildTime = trackBlock.getBuildTimeMs();

        TrackSingleGeometrySegment segment = new TrackSingleGeometrySegment(trackBlock, world, pos, 0, Gauge.standard());
        paths = new ArrayList<>();
        Map<Gauge, TrackSingleGeometrySegment> branch = new HashMap<>();
        branch.put(trackBlock.info.settings.gauge, segment);
        paths.add(branch);

//        tableIndex = ;
        switchState = trackBlock.isSwitchForced() ? trackBlock.info.switchForced.ordinal() : trackBlock.info.switchState.ordinal();
    }

    public void updateSwitchSate(SwitchState switchState) {
        this.switchState = switchState.ordinal();
    }

    public void updateTableState() {
        //todo
    }

    public TrackMultiGeometrySegment(ByteBuffer buffer) {
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
                branch.put(gauge, new TrackSingleGeometrySegment(buffer)); // single segment
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
}
