package cam72cam.immersiverailroading.gui;

import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.net.TrackRegionRequestPacket;
import cam72cam.immersiverailroading.render.util.Color;
import cam72cam.immersiverailroading.render.util.FlatCurveRenderer;
import cam72cam.immersiverailroading.track.BuilderCubicCurve;
import cam72cam.immersiverailroading.track.CubicCurve;
import cam72cam.immersiverailroading.track.VecYPR;
import cam72cam.immersiverailroading.track.graph.*;
import cam72cam.mod.MinecraftClient;
import cam72cam.mod.event.ClientEvents;
import cam72cam.mod.gui.helpers.GUIHelpers;
import cam72cam.mod.gui.screen.IScreen;
import cam72cam.mod.gui.screen.IScreenBuilder;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.render.opengl.RenderState;
import cam72cam.mod.world.World;
import util.Matrix4;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class GlobalRailwayMap implements IScreen {

    private static GlobalRailwayMap currentInstance;
    public static GlobalRailwayMap current() { return currentInstance; }

    private static final Color COLOR_GRID       = new Color(0.35, 0.35, 0.45, 0.7);
    private static final Color COLOR_GRID_MAJOR = new Color(0.55, 0.55, 0.65, 0.9);
    private static final Color COLOR_REGION     = new Color(1.00, 0.90, 0.20, 0.8);
    private static final Color COLOR_TRACK      = new Color(0.35, 0.90, 0.35, 1.0);
    private static final Color COLOR_TRACK_FB   = new Color(0.90, 0.60, 0.30, 1.0);
    private static final Color COLOR_PLAYER     = new Color(0.30, 0.60, 1.00, 1.0);

    private double panX, panZ;
    private double zoom = 2.0;

    private boolean dragging;
    private double dragStartScreenX, dragStartScreenY;
    private double dragStartPanX, dragStartPanZ;

    private WorldData data;
    private World world;

    private final Map<TrackSingleGeometrySegment, Vec3d> offsetCache = new HashMap<>();

    private int diagRegionCount, diagTrackCount, diagPointCount, diagNullCacheCount;
    private double diagTrackMinX, diagTrackMaxX;
    private double diagTrackMinZ, diagTrackMaxZ;

    // --- 请求控制 ---
    private boolean needsRequest = false;
    private long lastRequestTime = 0;
    private List<Long> lastRequestedRegions = new ArrayList<>();

    // ================================================================

    @Override
    public void init(IScreenBuilder builder) {
        currentInstance = this;
        world = MinecraftClient.getPlayer().getWorld();
        data = WorldData.get(world);

        Vec3d p = MinecraftClient.getPlayer().getPosition();
        panX = p.x;
        panZ = p.z;
        zoom = 2.0;

        offsetCache.clear();
        collectDiagnostics();

        // 刚打开时触发一次请求
        needsRequest = true;
    }

    @Override
    public void onClose() {
        if (currentInstance == this) currentInstance = null;
        dragging = false;
        offsetCache.clear();

        // 通知服务端释放该玩家地图所请求的 Region
        if (world != null) {
            new TrackRegionRequestPacket(world, Collections.emptyList()).sendToServer();
        }
    }

    private void collectDiagnostics() {
        diagRegionCount = 0;
        diagTrackCount = 0;
        diagPointCount = 0;
        diagNullCacheCount = 0;

        diagTrackMinX = Double.MAX_VALUE;
        diagTrackMinZ = Double.MAX_VALUE;
        diagTrackMaxX = -Double.MAX_VALUE;
        diagTrackMaxZ = -Double.MAX_VALUE;

        if (data == null) return;

        diagRegionCount = data.regions.size();
        for (TrackRegion region : data.regions.values()) {
            for (TrackMultiGeometrySegment seg : region.trackBlocks.values()) {
                for (Map<Gauge, TrackSingleGeometrySegment> branch : seg.paths) {
                    for (TrackSingleGeometrySegment single : branch.values()) {
                        diagTrackCount++;
                        Vec3i bp = single.getBlockPos();
                        diagTrackMinX = Math.min(diagTrackMinX, bp.x);
                        diagTrackMaxX = Math.max(diagTrackMaxX, bp.x);
                        diagTrackMinZ = Math.min(diagTrackMinZ, bp.z);
                        diagTrackMaxZ = Math.max(diagTrackMaxZ, bp.z);

                        List<VecYPR> points = single.pointsCache;
                        if (points == null || points.size() < 2) {
                            diagNullCacheCount++;
                        } else {
                            diagPointCount += points.size();
                        }
                    }
                }
            }
        }
    }

    // ================================================================
    // 鼠标事件
    // ================================================================

    public boolean handleMouse(ClientEvents.MouseGuiEvent evt) {
        int width  = GUIHelpers.getScreenWidth();
        int height = GUIHelpers.getScreenHeight();
        double cx = width / 2.0;
        double cy = height / 2.0;

        switch (evt.action) {
            case SCROLL: {
                double worldXBefore = panX + (evt.x - cx) / zoom;
                double worldZBefore = panZ + (evt.y - cy) / zoom;
                zoom = clamp(zoom * Math.pow(1.15, evt.scroll), 0.1, 64.0);
                panX = worldXBefore - (evt.x - cx) / zoom;
                panZ = worldZBefore - (evt.y - cy) / zoom;
                needsRequest = true;
                return false;
            }
            case CLICK: {
                dragging = true;
                dragStartScreenX = evt.x;
                dragStartScreenY = evt.y;
                dragStartPanX = panX;
                dragStartPanZ = panZ;
                return false;
            }
            case MOVE: {
                if (dragging) {
                    panX = dragStartPanX - (evt.x - dragStartScreenX) / zoom;
                    panZ = dragStartPanZ - (evt.y - dragStartScreenY) / zoom;
                    needsRequest = true;
                    return false;
                }
                return true;
            }
            case RELEASE: {
                if (dragging) {
                    dragging = false;
                    return false;
                }
                return true;
            }
        }
        return true;
    }

    // ================================================================
    // 绘制
    // ================================================================

    @Override
    public void draw(IScreenBuilder builder, RenderState state) {
        int width  = GUIHelpers.getScreenWidth();
        int height = GUIHelpers.getScreenHeight();

        GUIHelpers.drawRect(0, 0, width, height, 0xFF14141E);

        // 屏幕中心设为 (0,0) 原点
        RenderState mapState = state.clone().translate(width / 2.0, height / 2.0, 0);

        drawGrid(mapState, width, height);

        if (data != null) {
            drawRegionsAndTracks(mapState, width, height);
        }

        drawPlayerMarker(mapState, width, height);
        drawHud(width, height);

        // 处理网格/轨道视口请求
        handleRequests();
    }

    private void handleRequests() {
        if (!needsRequest) return;

        long now = System.currentTimeMillis();
        // 防抖：500ms 内最多请求一次
        if (now - lastRequestTime < 500) return;

        int width = GUIHelpers.getScreenWidth();
        int height = GUIHelpers.getScreenHeight();

        double halfW = (width / 2.0) / zoom;
        double halfH = (height / 2.0) / zoom;

        // 向外扩展一圈（大约 512 个方块，即一个 Region 的宽度）
        double extend = 512;
        int minX = (int) Math.floor(panX - halfW - extend);
        int maxX = (int) Math.ceil(panX + halfW + extend);
        int minZ = (int) Math.floor(panZ - halfH - extend);
        int maxZ = (int) Math.ceil(panZ + halfH + extend);

        int rxMin = minX >> 9;
        int rxMax = maxX >> 9;
        int rzMin = minZ >> 9;
        int rzMax = maxZ >> 9;

        // 安全限制：如果缩放到非常远，限制请求中心周围最多 15x15 个区块，防止撑爆网络
        int limit = 7;
        int centerRx = (int) panX >> 9;
        int centerRz = (int) panZ >> 9;
        rxMin = Math.max(rxMin, centerRx - limit);
        rxMax = Math.min(rxMax, centerRx + limit);
        rzMin = Math.max(rzMin, centerRz - limit);
        rzMax = Math.min(rzMax, centerRz + limit);

        List<Long> currentRegions = new ArrayList<>();
        for (int x = rxMin; x <= rxMax; x++) {
            for (int z = rzMin; z <= rzMax; z++) {
                currentRegions.add(((long) x << 32) | (z & 0xFFFFFFFFL));
            }
        }

        // 视口区域没变，或者已经发过了
        if (currentRegions.equals(lastRequestedRegions)) {
            needsRequest = false;
            return;
        }

        lastRequestedRegions = currentRegions;
        new TrackRegionRequestPacket(world, currentRegions).sendToServer();
        lastRequestTime = now;
        needsRequest = false;
    }

    private void drawGrid(RenderState state, int width, int height) {
        FlatCurveRenderer fr = new FlatCurveRenderer(state, null);

        double halfW = (width / 2.0) / zoom;
        double halfH = (height / 2.0) / zoom;
        double minX = panX - halfW, maxX = panX + halfW;
        double minZ = panZ - halfH, maxZ = panZ + halfH;

        double[] spacings = {1, 4, 16, 64, 256, 1024};
        double spacing = spacings[spacings.length - 1];
        for (double s : spacings) {
            if (s * zoom >= 24) { spacing = s; break; }
        }
        double majorSpacing = spacing * 4;
        double yScale = -zoom;

        double startX = Math.floor(minX / spacing) * spacing;
        for (double x = startX; x <= maxX; x += spacing) {
            if (Math.abs(x / majorSpacing - Math.round(x / majorSpacing)) < 1e-6) continue;
            fr.drawLine(new Vec3d(x - panX, minZ - panZ, 0), new Vec3d(x - panX, maxZ - panZ, 0), COLOR_GRID, zoom, yScale, 0.5);
        }
        double startZ = Math.floor(minZ / spacing) * spacing;
        for (double z = startZ; z <= maxZ; z += spacing) {
            if (Math.abs(z / majorSpacing - Math.round(z / majorSpacing)) < 1e-6) continue;
            fr.drawLine(new Vec3d(minX - panX, z - panZ, 0), new Vec3d(maxX - panX, z - panZ, 0), COLOR_GRID, zoom, yScale, 0.5);
        }

        startX = Math.floor(minX / majorSpacing) * majorSpacing;
        for (double x = startX; x <= maxX; x += majorSpacing) {
            fr.drawLine(new Vec3d(x - panX, minZ - panZ, 0), new Vec3d(x - panX, maxZ - panZ, 0), COLOR_GRID_MAJOR, zoom, yScale, 1);
        }
        startZ = Math.floor(minZ / majorSpacing) * majorSpacing;
        for (double z = startZ; z <= maxZ; z += majorSpacing) {
            fr.drawLine(new Vec3d(minX - panX, z - panZ, 0), new Vec3d(maxX - panX, z - panZ, 0), COLOR_GRID_MAJOR, zoom, yScale, 1);
        }
    }

    private void drawRegionsAndTracks(RenderState state, int width, int height) {
        FlatCurveRenderer fr = new FlatCurveRenderer(state, null);

        double halfW = (width / 2.0) / zoom;
        double halfH = (height / 2.0) / zoom;
        double minX = panX - halfW, maxX = panX + halfW;
        double minZ = panZ - halfH, maxZ = panZ + halfH;
        double yScale = -zoom;

        // Region 边框
        for (Map.Entry<Long, TrackRegion> entry : data.regions.entrySet()) {
            long regionId = entry.getKey();
            int rx = TrackRegionUtil.regionX(regionId);
            int rz = TrackRegionUtil.regionZ(regionId);
            int rMinX = rx << 9;
            int rMinZ = rz << 9;
            int rMaxX = rMinX + 512;
            int rMaxZ = rMinZ + 512;

            if (rMaxX < minX || rMinX > maxX || rMaxZ < minZ || rMinZ > maxZ) continue;

            fr.drawLine(new Vec3d(rMinX - panX, rMinZ - panZ, 0), new Vec3d(rMaxX - panX, rMinZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
            fr.drawLine(new Vec3d(rMaxX - panX, rMinZ - panZ, 0), new Vec3d(rMaxX - panX, rMaxZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
            fr.drawLine(new Vec3d(rMaxX - panX, rMaxZ - panZ, 0), new Vec3d(rMinX - panX, rMaxZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
            fr.drawLine(new Vec3d(rMinX - panX, rMaxZ - panZ, 0), new Vec3d(rMinX - panX, rMinZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
        }

        // 轨道
        for (TrackRegion region : data.regions.values()) {
            for (TrackMultiGeometrySegment segment : region.trackBlocks.values()) {
                for (Map<Gauge, TrackSingleGeometrySegment> branch : segment.paths) {
                    for (TrackSingleGeometrySegment single : branch.values()) {
                        drawSingleSegment(fr, single, panX, panZ, minX, maxX, minZ, maxZ, yScale);
                    }
                }
            }
        }
    }

    private void drawSingleSegment(FlatCurveRenderer fr, TrackSingleGeometrySegment single,
                                   double panX, double panZ,
                                   double minX, double maxX, double minZ, double maxZ, double yScale) {
        List<VecYPR> points = single.pointsCache;
        Vec3d offset = offsetCache.get(single);
        BuilderCubicCurve builder = null;

        if (points != null && points.size() >= 2) {
            if (offset == null) {
                offset = computeOffset(single);
                offsetCache.put(single, offset);
            }
            double ox = offset.x, oz = offset.z;
            for (int i = 0; i < points.size() - 1; i++) {
                VecYPR a = points.get(i);
                VecYPR b = points.get(i + 1);
                double ax = a.x + ox, az = a.z + oz;
                double bx = b.x + ox, bz = b.z + oz;
                if (!segmentVisible(ax, az, bx, bz, minX, maxX, minZ, maxZ)) continue;
                fr.drawLine(new Vec3d(ax - panX, az - panZ, 0), new Vec3d(bx - panX, bz - panZ, 0), COLOR_TRACK, zoom, yScale, 1);
            }
            return;
        }

        try {
            builder = single.getAndUpdateBuilder(world);
            if (offset == null) {
                offset = new Vec3d(single.getBlockPos()).add(builder.info.placementInfo.placementPosition);
                offsetCache.put(single, offset);
            }
            CubicCurve curve = builder.getCurve();
            if (curve == null) return;

            double ox = offset.x, oz = offset.z;
            final int N = 32;
            Vec3d prev = curve.position(0);
            for (int i = 1; i <= N; i++) {
                Vec3d curr = curve.position((double) i / N);
                double ax = prev.x + ox, az = prev.z + oz;
                double bx = curr.x + ox, bz = curr.z + oz;
                if (segmentVisible(ax, az, bx, bz, minX, maxX, minZ, maxZ)) {
                    fr.drawLine(new Vec3d(ax - panX, az - panZ, 0), new Vec3d(bx - panX, bz - panZ, 0), COLOR_TRACK_FB, zoom, yScale, 1);
                }
                prev = curr;
            }
        } catch (Exception ignore) {}
    }

    private Vec3d computeOffset(TrackSingleGeometrySegment single) {
        try {
            BuilderCubicCurve builder = single.getAndUpdateBuilder(world);
            return new Vec3d(single.getBlockPos()).add(builder.info.placementInfo.placementPosition);
        } catch (Exception e) {
            return new Vec3d(single.getBlockPos());
        }
    }

    private void drawPlayerMarker(RenderState state, int width, int height) {
        Vec3d p = MinecraftClient.getPlayer().getPosition();
        double dx = p.x - panX;
        double dy = p.z - panZ;
        double size = 4;
        double lineWidth = 1;

        double w = size / zoom;
        double h = -size / zoom;
        double yScale = -zoom;

        Vec3d top    = new Vec3d(dx,      dy + h, 0);
        Vec3d right  = new Vec3d(dx + w,  dy,      0);
        Vec3d bottom = new Vec3d(dx,      dy - h, 0);
        Vec3d left   = new Vec3d(dx - w,  dy,      0);

        FlatCurveRenderer fr = new FlatCurveRenderer(state, null);

        drawExtendedLine(fr, top, right, lineWidth, zoom, yScale);
        drawExtendedLine(fr, right, bottom, lineWidth, zoom, yScale);
        drawExtendedLine(fr, bottom, left, lineWidth, zoom, yScale);
        drawExtendedLine(fr, left, top, lineWidth, zoom, yScale);
    }

    private void drawExtendedLine(FlatCurveRenderer fr, Vec3d start, Vec3d end, double lineWidth, double xScale, double yScale) {
        Vec3d dir = end.subtract(start).normalize();
        double extLen = (lineWidth / 2.0) / Math.abs(xScale);
        Vec3d extStart = start.subtract(dir.scale(extLen));
        Vec3d extEnd = end.add(dir.scale(extLen));
        fr.drawLine(extStart, extEnd, COLOR_PLAYER, xScale, yScale, lineWidth);
    }

    private void drawHud(int width, int height) {
        Vec3d p = MinecraftClient.getPlayer().getPosition();
        Matrix4 m = new Matrix4();

        int y = 10;
        GUIHelpers.drawCenteredString(String.format("Zoom: %.2fx", zoom),
                width / 2, y, 0xFFFFFFFF, m); y += 14;
        GUIHelpers.drawCenteredString(
                String.format("Center: %d, %d   Player: %d, %d",
                        (int) panX, (int) panZ, (int) p.x, (int) p.z),
                width / 2, y, 0xFFFFFFFF, m); y += 14;

        GUIHelpers.drawCenteredString(
                String.format("Regions=%d  Tracks=%d  Points=%d  NullCache=%d",
                        diagRegionCount, diagTrackCount, diagPointCount, diagNullCacheCount),
                width / 2, y, 0xFFFFFF80, m); y += 14;

        if (diagTrackCount > 0) {
            GUIHelpers.drawCenteredString(
                    String.format("TrackBBox: X[%d,%d] Z[%d,%d]",
                            (int) diagTrackMinX, (int) diagTrackMaxX,
                            (int) diagTrackMinZ, (int) diagTrackMaxZ),
                    width / 2, y, 0xFFFFFF80, m);
        } else {
            GUIHelpers.drawCenteredString(
                    "No track data on client yet",
                    width / 2, y, 0xFFFF6060, m);
        }

        GUIHelpers.drawCenteredString(
                "Scroll = zoom   Drag = pan   ESC = close",
                width / 2, height - 16, 0xFFA0A0A0, m);
    }

    private boolean segmentVisible(double wx1, double wz1, double wx2, double wz2,
                                   double minX, double maxX, double minZ, double maxZ) {
        double sMinX = Math.min(wx1, wx2), sMaxX = Math.max(wx1, wx2);
        double sMinZ = Math.min(wz1, wz2), sMaxZ = Math.max(wz1, wz2);
        return !(sMaxX < minX || sMinX > maxX || sMaxZ < minZ || sMinZ > maxZ);
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}