package cam72cam.immersiverailroading.track.graph;

import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrackRegion {
    public final Map<Vec3i, TrackMultiGeometrySegment> trackBlocks;// fake final
    // key: gag region-relative pos, value: parent track block region-relative pos list
    public final HashMap<Vec3i, List<Vec3i>> trackBlockParents;

    boolean needsWriteToDisk;
    boolean dirty;

    public TrackRegion() {
        trackBlocks = new HashMap<>();
        trackBlockParents = new HashMap<>();
    }

    // region is 512 * worldHeight * 512, only x/z are limited
    public static Vec3i toRegionBlockPos(Vec3i blockPos) {
        return new Vec3i(blockPos.x & 0x1FF, blockPos.y, blockPos.z & 0x1FF);
    }

    public static Vec3i toBlockPos(long regionPos, Vec3i regionBlockPos) {
        int regionX = (int) (regionPos >> 32);
        int regionZ = (int) regionPos;
        return new Vec3i(
                (regionX << 9) + regionBlockPos.x,
                regionBlockPos.y,
                (regionZ << 9) + regionBlockPos.z
        );
    }

    public TrackRegion(ByteBuffer buffer, World world) {
        int version = buffer.getInt();
        if (version != 1) {
            throw new RuntimeException(String.format("Invalid track block data version %d", version));
        }

        int size = buffer.getInt();
        trackBlocks = new HashMap<>(size);
        trackBlockParents = new HashMap<>(size);
        for (int i = 0; i < size; i++) {
            int x = buffer.getInt();
            int y = buffer.getInt();
            int z = buffer.getInt();
            Vec3i relBlockPos = new Vec3i(x, y, z);
            TrackMultiGeometrySegment segment = new TrackMultiGeometrySegment(buffer, world, relBlockPos);
            trackBlocks.put(relBlockPos, segment);

            for (Vec3i offset : segment.getPositionsCache()) {
                Vec3i gagPos = toRegionBlockPos(relBlockPos.add(offset));
                List<Vec3i> parents = trackBlockParents.getOrDefault(gagPos, new ArrayList<>());
                parents.add(relBlockPos);
                trackBlockParents.put(gagPos, parents);
            }
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
                Vec3i pos = entry.getKey();// rel pos in region
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
            return trackBlocks.get(toRegionBlockPos(pos));
        }
    }

    public boolean isEmpty() {
        synchronized (trackBlocks) {
            return trackBlocks.isEmpty();
        }
    }

    public boolean removeTrackBlock(Vec3i pos) {
        synchronized (trackBlocks) {
            Vec3i relPos = toRegionBlockPos(pos);
            TrackMultiGeometrySegment removed = trackBlocks.remove(relPos);
            if (removed == null) {
                return false;
            }
            for (Vec3i offset : removed.getPositionsCache()) {
                Vec3i gagPos = toRegionBlockPos(relPos.add(offset));
                List<Vec3i> parents = trackBlockParents.get(gagPos);
                if (parents != null) {
                    parents.remove(relPos);
                    if (parents.isEmpty()) {
                        trackBlockParents.remove(gagPos);
                    }
                }
            }
            needsWriteToDisk = true;
            dirty = true;
            return true;
        }
    }

    public void setTrackBlock(Vec3i pos, TrackMultiGeometrySegment block) {
        synchronized (trackBlocks) {
            Vec3i relBlockPos = toRegionBlockPos(pos);
            trackBlocks.put(relBlockPos, block);

            for (Vec3i offset : block.getPositionsCache()) {
                Vec3i gagPos = toRegionBlockPos(relBlockPos.add(offset));
                List<Vec3i> parents = trackBlockParents.getOrDefault(gagPos, new ArrayList<>());
                parents.add(relBlockPos);
                trackBlockParents.put(gagPos, parents);
            }

            needsWriteToDisk = true;
            dirty = true;
        }
    }
}