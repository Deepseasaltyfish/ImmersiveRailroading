package cam72cam.immersiverailroading.render.util;

import cam72cam.immersiverailroading.ImmersiveRailroading;
import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.render.ExpireableMap;
import cam72cam.immersiverailroading.render.rail.RailRender;
import cam72cam.immersiverailroading.track.CubicCurve;
import cam72cam.immersiverailroading.track.VecYPR;
import cam72cam.immersiverailroading.track.graph.*;
import cam72cam.immersiverailroading.util.RailInfo;
import cam72cam.mod.MinecraftClient;
import cam72cam.mod.entity.Player;
import cam72cam.mod.item.ItemStack;
import cam72cam.mod.math.Matrix3;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.render.GlobalRender;
import cam72cam.mod.render.opengl.*;
import cam72cam.mod.text.PlayerMessage;
import cam72cam.mod.world.World;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static cam72cam.immersiverailroading.render.util.FlatCurveRenderer.lineImg;
//todo: 和BezierCurveRenderer合并
public class VolumetricCurveRenderer {
    private static ExpireableMap<String, RailInfo> infoCache = new ExpireableMap<>();

    public static void drawBlockWireFrame(Vec3d pos, RenderState state, Color color, double size, double lineWidth) {
        DirectDraw draw = new DirectDraw();
        drawBlockWireFrame(draw, new VecYPR(pos, 0), color, size, lineWidth);
        draw.draw(state.clone().texture(Texture.wrap(lineImg))
                .alpha_test(false)
                .blend(new BlendMode(BlendMode.GL_SRC_ALPHA, BlendMode.GL_ONE_MINUS_SRC_ALPHA)));
    }

    public static void drawBlockWireFrame(DirectDraw draw, VecYPR pos, Color color, double size, double lineWidth) {
        double inset = size * 0.002;
        double half = size / 2.0;
        double h0 = -half + inset;
        double h1 = half - inset;

        // 以原点为中心的 8 个局部角点
        Vec3d[] local = {
                new Vec3d(h0, h0, h0),
                new Vec3d(h1, h0, h0),
                new Vec3d(h1, h0, h1),
                new Vec3d(h0, h0, h1),
                new Vec3d(h0, h1, h0),
                new Vec3d(h1, h1, h0),
                new Vec3d(h1, h1, h1),
                new Vec3d(h0, h1, h1),
        };

        // 应用 yaw/pitch/roll 再平移到世界位置
        Matrix3 m = pos.toMatrix3();
        Vec3d[] c = new Vec3d[8];
        for (int i = 0; i < 8; i++) {
            Vec3d r = m.apply(local[i]); // 若 Matrix3 的乘法方法名不是 mul，改成对应的
            c[i] = new Vec3d(pos.x + r.x, pos.y + r.y, pos.z + r.z);
        }

        int[][] edges = {
                {0, 1}, {1, 2}, {2, 3}, {3, 0},
                {4, 5}, {5, 6}, {6, 7}, {7, 4},
                {0, 4}, {1, 5}, {2, 6}, {3, 7},
        };

        for (int[] e : edges) {
            appendLineQuad(draw, c[e[0]], c[e[1]], color, lineWidth);
        }
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

                    drawBlockWireFrame(info.placementInfo.placementPosition.scale(-1).add(0.5,0.5,0.5), currentState, Color.YELLOW, 1, 1/16f);
                    renderCurve(single.getValue().pointsCache, Color.LIME, 1 / 16f, currentState.clone().translate(0, 1, 0));
                    renderHandles(single.getValue().baseCurve, Color.MAGENTA, Color.CYAN, Color.RED, 1 / 16f, 1 / 4f, currentState.clone().translate(0, 1 + 1 / 16f, 0));

                    TrackNode node = new TrackNode(single.getValue(), true, world, blockPos);
                    TrackNode next = node.getConn(world, blockPos, false);
                    if(next != null) {
                        List<VecYPR> conn = new ArrayList<>();
                        conn.add(node.point);
                        conn.add(next.point);
                        renderCurve(conn, Color.ORANGE, 1/16f, state.clone().translate(0, 1 + 1/32f, 0));
                    }

                    TrackNode node2 = new TrackNode(single.getValue(), false, world, blockPos);
                    TrackNode next2 = node2.getConn(world, blockPos, true);
                    if(next2 != null) {
                        List<VecYPR> conn2 = new ArrayList<>();
                        conn2.add(node2.point);
                        conn2.add(next2.point);
                        renderCurve(conn2, Color.ORANGE, 1/16f, state.clone().translate(0, 1 + 1/32f, 0));
                    }

//                  renderDebug(info, currentState);
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

    public static void renderCurve(List<VecYPR> points, Color color, double width, RenderState state) {
        if (points == null || points.size() < 2) return;

        DirectDraw draw = new DirectDraw();
        for (int i = 0; i < points.size() - 1; i++) {
            appendLineQuad(draw, points.get(i), points.get(i + 1), color, width);
        }

        draw.draw(state.clone().texture(Texture.wrap(lineImg))
                .alpha_test(false)
                .blend(new BlendMode(BlendMode.GL_SRC_ALPHA, BlendMode.GL_ONE_MINUS_SRC_ALPHA)));
    }

    public static void renderHandles(CubicCurve curve, Color handleColor, Color pointColor, Color controlColor,
                                     double width, double pointSize, RenderState state) {
        DirectDraw draw = new DirectDraw();

        // 两条控制柄连线：p1 -> ctrl1，p2 -> ctrl2
        appendLineQuad(draw, curve.p1, curve.ctrl1, handleColor, width);
        appendLineQuad(draw, curve.p2, curve.ctrl2, handleColor, width);

        // 四个点：p1、ctrl1、ctrl2、p2
        drawBlockWireFrame(draw, new VecYPR(curve.p1.add(0, -1/32f, 0), 45, 0, 45), pointColor, pointSize, 1/16f);
        drawBlockWireFrame(draw, new VecYPR(curve.ctrl1.add(0, -1/32f, 0), 45, 0, 45), controlColor, pointSize, 1/16f);
        drawBlockWireFrame(draw, new VecYPR(curve.ctrl2.add(0, -1/32f, 0), 45, 0, 45), controlColor, pointSize, 1/16f);
        drawBlockWireFrame(draw, new VecYPR(curve.p2.add(0, -1/32f, 0), 45, 0, 45), pointColor, pointSize, 1/16f);

        draw.draw(state.clone().texture(Texture.wrap(lineImg))
                .alpha_test(false)
                .blend(new BlendMode(BlendMode.GL_SRC_ALPHA, BlendMode.GL_ONE_MINUS_SRC_ALPHA)));
    }
}
