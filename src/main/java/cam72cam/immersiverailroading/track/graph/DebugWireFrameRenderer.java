package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.gui.util.Color;
import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.render.ExpireableMap;
import cam72cam.immersiverailroading.render.rail.RailRender;
import cam72cam.immersiverailroading.util.RailInfo;
import cam72cam.mod.MinecraftClient;
import cam72cam.mod.entity.Player;
import cam72cam.mod.item.ItemStack;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.render.GlobalRender;
import cam72cam.mod.render.opengl.*;
import cam72cam.mod.world.World;

import java.util.Map;

import static cam72cam.immersiverailroading.gui.util.BezierRenderer.lineImg;

public class DebugWireFrameRenderer {
    private static ExpireableMap<String, RailInfo> infoCache = new ExpireableMap<>();

    public static void drawBlockWireFrame(Vec3d pos, RenderState state) {
        Color color = new Color(0, 1, 0, 0.9999f);
        double lineWidth = 1.0 / 16.0;
        double inset = 0.002;

        double x0 = pos.x + inset;
        double y0 = pos.y + inset;
        double z0 = pos.z + inset;
        double x1 = pos.x + 1.0 - inset;
        double y1 = pos.y + 1.0 - inset;
        double z1 = pos.z + 1.0 - inset;

        // 8 个角点
        Vec3d[] c = {
                new Vec3d(x0, y0, z0), // 0: ---
                new Vec3d(x1, y0, z0), // 1: +--
                new Vec3d(x1, y0, z1), // 2: +-+
                new Vec3d(x0, y0, z1), // 3: --+
                new Vec3d(x0, y1, z0), // 4: -+-
                new Vec3d(x1, y1, z0), // 5: ++-
                new Vec3d(x1, y1, z1), // 6: +++
                new Vec3d(x0, y1, z1), // 7: -++
        };

        // 12 条棱
        int[][] edges = {
                {0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底面
                {4, 5}, {5, 6}, {6, 7}, {7, 4}, // 顶面
                {0, 4}, {1, 5}, {2, 6}, {3, 7}, // 竖棱
        };

        DirectDraw draw = new DirectDraw();
        for (int[] e : edges) {
            appendLineQuad(draw, c[e[0]], c[e[1]], color, lineWidth);
        }

        draw.draw(state.clone().texture(Texture.wrap(lineImg))
                .alpha_test(false)
                .blend(new BlendMode(BlendMode.GL_SRC_ALPHA, BlendMode.GL_ONE_MINUS_SRC_ALPHA)));
    }

    private static void appendLineQuad(DirectDraw draw, Vec3d start, Vec3d end, Color color, double width) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double dz = end.z - start.z;
        double len = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-6) return;

        double ux = dx / len;
        double uy = dy / len;
        double uz = dz / len;

        // 选一个和 dir 不平行的 "up"，避免叉积为 0
        double upx, upy, upz;
        if (Math.abs(uy) > 0.9) {
            upx = 1; upy = 0; upz = 0;
        } else {
            upx = 0; upy = 1; upz = 0;
        }

        // n1 = dir × up
        double n1x = uy * upz - uz * upy;
        double n1y = uz * upx - ux * upz;
        double n1z = ux * upy - uy * upx;
        double n1Len = Math.sqrt(n1x * n1x + n1y * n1y + n1z * n1z);
        if (n1Len < 1e-6) return;
        n1x /= n1Len; n1y /= n1Len; n1z /= n1Len;

        // n2 = dir × n1，和 n1 垂直
        double n2x = uy * n1z - uz * n1y;
        double n2y = uz * n1x - ux * n1z;
        double n2z = ux * n1y - uy * n1x;
        // n2 已经是单位向量（dir 和 n1 都归一化且垂直），不用再归一化

        double half = width / 2;

        // 两片：分别沿 n1 和 n2 展开
        appendQuad(draw, start, end, n1x * half, n1y * half, n1z * half, color);
        appendQuad(draw, start, end, n2x * half, n2y * half, n2z * half, color);
    }

    private static void appendQuad(DirectDraw draw, Vec3d start, Vec3d end,
                                   double ox, double oy, double oz, Color color) {
        Vec3d p1 = new Vec3d(start.x + ox, start.y + oy, start.z + oz);
        Vec3d p2 = new Vec3d(start.x - ox, start.y - oy, start.z - oz);
        Vec3d p3 = new Vec3d(end.x   - ox, end.y   - oy, end.z   - oz);
        Vec3d p4 = new Vec3d(end.x   + ox, end.y   + oy, end.z   + oz);

        draw.vertex(p1).color(color.r(), color.g(), color.b(), color.a()).uv(0, 0);
        draw.vertex(p2).color(color.r(), color.g(), color.b(), color.a()).uv(0, 1);
        draw.vertex(p3).color(color.r(), color.g(), color.b(), color.a()).uv(1, 1);
        draw.vertex(p4).color(color.r(), color.g(), color.b(), color.a()).uv(1, 0);
    }

    public static void renderWireFrame(RenderState state, float partialTicks) {
        World world = MinecraftClient.getPlayer().getWorld();

        if(WorldData.get(world) == null) return;

        state.blend(new BlendMode(BlendMode.GL_CONSTANT_ALPHA, BlendMode.GL_ONE).constantColor(1, 1, 1, 1f)).lightmap(1, 1);

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

                    currentState.translate(new Vec3d(blockPos).add(info.placementInfo.placementPosition));

                    drawBlockWireFrame(info.placementInfo.placementPosition.scale(-1), currentState);
                    renderDebug(info, currentState);
                }
            }
        }
    }

    public static void renderMouseOver(Player player, ItemStack stack, Vec3i vec3i, Vec3d vec3d, RenderState state, float partialTicks) {
        state.translate(GlobalRender.getCameraPos(partialTicks).scale(-1));
        renderWireFrame(state, partialTicks);
    }

    public static void renderDebug(RailInfo info, RenderState state) {
        state.lighting(false);
        RailRender renderer = RailRender.get(info);
        Vec3d off = info.placementInfo.placementPosition;

        MinecraftClient.startProfiler("rail");
        renderer.renderRailModel(state);
        MinecraftClient.endProfiler();

        state.translate(-off.x, -off.y, -off.z);

        MinecraftClient.startProfiler("base");
        renderer.renderRailBase(state);
        MinecraftClient.endProfiler();
    }
}
