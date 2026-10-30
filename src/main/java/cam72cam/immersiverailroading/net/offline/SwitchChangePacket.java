package cam72cam.immersiverailroading.net.offline;

import cam72cam.immersiverailroading.library.SwitchState;
import cam72cam.immersiverailroading.library.TrackItems;
import cam72cam.immersiverailroading.track.graph.TrackMultiGeometrySegment;
import cam72cam.immersiverailroading.track.graph.WorldData;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.net.Packet;
import cam72cam.mod.serialization.TagField;
import cam72cam.mod.world.World;

public class SwitchChangePacket extends Packet {
    @TagField
    private World world;
    @TagField
    private Vec3i pos;
    @TagField(typeHint = SwitchState.class)
    private SwitchState targetState;

    public SwitchChangePacket() {
        // Reflection
    }

    public SwitchChangePacket(World world, Vec3i pos, SwitchState targetState) {
        this.world = world;
        this.pos = pos;
        this.targetState = targetState;
    }

    @Override
    protected void handle() {
        WorldData data = WorldData.get(world);
        if (data == null) return;

        // 这里用绝对坐标去找道岔。因为 TrackMultiGeometrySegment 的 getBlockPos 就是返回父级绝对坐标
        TrackMultiGeometrySegment seg = data.getTrackBlock(pos);
        if (seg != null && seg.type == TrackItems.SWITCH) {
            // 1. 更新状态
            seg.updateSwitchSate(targetState);
        }
    }
}