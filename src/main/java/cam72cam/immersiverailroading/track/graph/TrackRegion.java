package cam72cam.immersiverailroading.track.graph;

import cam72cam.mod.math.Vec3i;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

public class TrackRegion {//todo 按照chunk pos/pos排序？ hashset?
    private final Map<Vec3i, TrackMultiGeometrySegment> trackBlocks;
    boolean needsWriteToDisk;
    boolean dirty;

    public TrackRegion() {
        trackBlocks = new HashMap<>();
    }

    public TrackRegion(ByteBuffer buffer) {
        int version = buffer.getInt();
        if (version != 1) {
            throw new RuntimeException(String.format("Invalid track block data version %d", version));
        }

        int size = buffer.getInt();
        trackBlocks = new HashMap<>(size);
        for (int i = 0; i < size; i++) {
            int x = buffer.getInt();
            int y = buffer.getInt();
            int z = buffer.getInt();
            trackBlocks.put(new Vec3i(x, y, z), new TrackMultiGeometrySegment(buffer));
        }
    }

    private int sizeBytes() {
        int bytes = 0;
        bytes += Integer.BYTES; // version
        bytes += Integer.BYTES; // trackBlocks.size()
        for (TrackMultiGeometrySegment value : trackBlocks.values()) {
            bytes += Integer.BYTES * 3; // pos.xyz
            bytes += value.sizeBytes(); // trackBlock.write()
        }

        return bytes;
    }

    public ByteBuffer write() {
        // Could be faster with a CoW, not sure how long this blocking is
        synchronized (trackBlocks) {
            ByteBuffer buffer = ByteBuffer.allocate(sizeBytes());

            buffer.putInt(1); // version
            buffer.putInt(trackBlocks.size());
            for (Map.Entry<Vec3i, TrackMultiGeometrySegment> entry : trackBlocks.entrySet()) {
                Vec3i pos = entry.getKey();
                buffer.putInt(pos.x);
                buffer.putInt(pos.y);
                buffer.putInt(pos.z);
                entry.getValue().write(buffer);
            }
            return buffer;
        }
    }

    public TrackMultiGeometrySegment getTrackBlock(Vec3i pos) {
        synchronized (trackBlocks) {
            return trackBlocks.get(pos);
        }
    }

    public void setTrackBlock(Vec3i pos, TrackMultiGeometrySegment block) {
        synchronized (trackBlocks) {
            trackBlocks.put(pos, block);
            needsWriteToDisk = true;
            dirty = true;
        }
    }
}
