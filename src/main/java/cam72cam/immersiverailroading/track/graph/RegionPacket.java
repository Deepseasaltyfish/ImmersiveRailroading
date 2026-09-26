package cam72cam.immersiverailroading.track.graph;

import cam72cam.mod.net.Packet;
import cam72cam.mod.serialization.TagField;
import cam72cam.mod.world.World;

import java.nio.ByteBuffer;

public class RegionPacket extends Packet {
    @TagField
    private World world;
    @TagField
    private long id;
    public RegionPacket() {
        // Reflection
    }
    public RegionPacket(World world, long id, TrackRegion region) {
        this.world = world;
        this.id = id;
        // This is stupidly inefficient
        // TODO LZ4 compress?
        this.raw = region.write().array();
    }

    @Override
    protected void handle() {
        TrackRegion region = new TrackRegion(ByteBuffer.wrap(raw));

        WorldData data = WorldData.getOrCreate(world);
        if (data != null) {
            data.regions.put(id, region);
        }
    }
}
