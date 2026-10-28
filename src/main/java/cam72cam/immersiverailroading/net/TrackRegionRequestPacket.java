package cam72cam.immersiverailroading.net;

import cam72cam.immersiverailroading.track.graph.WorldData;
import cam72cam.mod.net.Packet;
import cam72cam.mod.serialization.TagField;
import cam72cam.mod.world.World;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TrackRegionRequestPacket extends Packet {
    @TagField
    private World world;

    public TrackRegionRequestPacket() {
        // Reflection
    }

    public TrackRegionRequestPacket(World world, List<Long> requestRegions) {
        this.world = world;

        // 仿照 TrackRegionDeltaPacket，直接转成 byte[] 存入基类的 raw 字段
        if (requestRegions == null || requestRegions.isEmpty()) {
            this.raw = new byte[0];
        } else {
            ByteBuffer buf = ByteBuffer.allocate(requestRegions.size() * Long.BYTES);
            for (Long id : requestRegions) {
                buf.putLong(id);
            }
            this.raw = buf.array();
        }
    }

    @Override
    protected void handle() {
        WorldData data = WorldData.getOrCreate(world);
        if (data == null) return;

        // 收到空包，说明客户端关闭了地图
        if (raw == null || raw.length == 0) {
            data.handleClientMapRequest(getPlayer(), Collections.emptyList());
            return;
        }

        // 反序列化字节流为 List<Long>
        int count = raw.length / Long.BYTES;
        List<Long> requestRegions = new ArrayList<>(count);
        ByteBuffer buf = ByteBuffer.wrap(raw);
        for (int i = 0; i < count; i++) {
            requestRegions.add(buf.getLong());
        }

        data.handleClientMapRequest(getPlayer(), requestRegions);
    }
}