package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.render.ExpireableMap;
import cam72cam.immersiverailroading.render.rail.RailRender;
import cam72cam.immersiverailroading.util.RailInfo;
import cam72cam.mod.entity.Player;
import cam72cam.mod.item.ItemStack;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.render.GlobalRender;
import cam72cam.mod.render.opengl.BlendMode;
import cam72cam.mod.render.opengl.RenderState;
import cam72cam.mod.world.World;

import java.util.Map;

public class DebugWireFrameRenderer {
    private static ExpireableMap<String, RailInfo> infoCache = new ExpireableMap<>();
    public static void renderWireFrame(Player player, ItemStack stack, Vec3i pos, Vec3d vec, RenderState state, float partialTicks) {
        World world = player.getWorld();

        state.blend(new BlendMode(BlendMode.GL_CONSTANT_ALPHA, BlendMode.GL_ONE).constantColor(1, 1, 1, 0.5f)).lightmap(1, 1);

        Map<Long, TrackRegion> regions = WorldData.get(world).regions;
        for(Map.Entry<Long, TrackRegion> region : regions.entrySet()) {
            for(Map.Entry<Vec3i, TrackMultiGeometrySegment> segment : region.getValue().trackBlocks.entrySet()){
                for(Map.Entry<Gauge, TrackSingleGeometrySegment> single : segment.getValue().paths.getFirst().entrySet()){
                    Vec3i blockPos = TrackRegion.toBlockPos(region.getKey(), segment.getKey());

                    RailInfo info = single.getValue().getAndUpdateBuilder(world, blockPos).info;

                    String key = info.uniqueID + info.placementInfo.placementPosition;
                    RailInfo cached = infoCache.get(key);
                    if (cached != null) {
                        info = cached;
                    } else {
                        infoCache.put(key, info);
                    }

                    RenderState currentState = state.clone();
                    Vec3d cameraPos = GlobalRender.getCameraPos(partialTicks);
                    currentState.translate(new Vec3d(blockPos).add(info.placementInfo.placementPosition).subtract(cameraPos));

                    RailRender.renderDebug(info, currentState);
                }
            }
        }
    }
}
