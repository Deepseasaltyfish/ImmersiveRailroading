package cam72cam.immersiverailroading.gui;

import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.library.GuiText;
import cam72cam.immersiverailroading.library.SwitchState;
import cam72cam.immersiverailroading.library.TrackItems;
import cam72cam.immersiverailroading.net.offline.SwitchChangePacket;
import cam72cam.immersiverailroading.net.offline.TrackRegionRequestPacket;
import cam72cam.immersiverailroading.net.offline.VehicleRequestPacket;
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
    private static final Color COLOR_VEHICLE    = new Color(1.00, 0.30, 0.30, 1.0);

    // 道岔交互颜色
    private static final Color COLOR_SWITCH_HOVER  = new Color(1.0, 0.8, 0.2, 1.0);
    private static final Color COLOR_SWITCH_SELECT = new Color(0.2, 1.0, 1.0, 1.0);
    private static final int COLOR_BTN_BG          = 0xFF333333;
    private static final int COLOR_BTN_HOVER       = 0xFF555555;
    private static final int COLOR_BTN_CLOSE       = 0xFF882222;
    private static final int COLOR_BTN_CLOSE_HOVER = 0xFFAA3333;
    private static final int COLOR_BTN_TEXT        = 0xFFFFFFFF;

    private double panX, panZ;
    private double zoom = 2.0;

    private boolean dragging;
    private double dragStartScreenX, dragStartScreenY;
    private double dragStartPanX, dragStartPanZ;

    private WorldData data;
    private World world;

    private final Map<TrackSingleGeometrySegment, Vec3d> offsetCache = new HashMap<>();
    private int diagRegionCount, diagTrackCount, diagPointCount, diagNullCacheCount;
    private double diagTrackMinX, diagTrackMaxX, diagTrackMinZ, diagTrackMaxZ;

    private int tickCounter = 0;

    // 交互状态
    private TrackMultiGeometrySegment hoveredSwitch = null;
    private TrackMultiGeometrySegment selectedSwitch = null;

    // 用于 UI 绘制的鼠标坐标缓存（因为 draw 时无法直接获取实时鼠标）
    private int lastMouseX = 0;
    private int lastMouseY = 0;

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
        tickCounter = 0;
    }

    @Override
    public void onClose() {
        if (currentInstance == this) currentInstance = null;
        dragging = false;
        offsetCache.clear();
        if (world != null) {
            new TrackRegionRequestPacket(world, Collections.emptyList()).sendToServer();
        }
    }

    public static void onClientTick() {
        if (currentInstance != null) {
            currentInstance.tick();
        }
    }

    private void tick() {
        tickCounter++;
        if (tickCounter >= 20) {
            tickCounter = 0;
            sendViewportRequests();
        }
    }

    private void sendViewportRequests() {
        int width = GUIHelpers.getScreenWidth();
        int height = GUIHelpers.getScreenHeight();
        double halfW = (width / 2.0) / zoom;
        double halfH = (height / 2.0) / zoom;

        int minX = (int) Math.floor(panX - halfW - 512);
        int maxX = (int) Math.ceil(panX + halfW + 512);
        int minZ = (int) Math.floor(panZ - halfH - 512);
        int maxZ = (int) Math.ceil(panZ + halfH + 512);

        int rxMin = minX >> 9, rxMax = maxX >> 9, rzMin = minZ >> 9, rzMax = maxZ >> 9;
        int limit = 7;
        int centerRx = (int) panX >> 9, centerRz = (int) panZ >> 9;
        rxMin = Math.max(rxMin, centerRx - limit); rxMax = Math.min(rxMax, centerRx + limit);
        rzMin = Math.max(rzMin, centerRz - limit); rzMax = Math.min(rzMax, centerRz + limit);

        List<Long> currentRegions = new ArrayList<>();
        for (int x = rxMin; x <= rxMax; x++) {
            for (int z = rzMin; z <= rzMax; z++) {
                currentRegions.add(((long) x << 32) | (z & 0xFFFFFFFFL));
            }
        }
        new TrackRegionRequestPacket(world, currentRegions).sendToServer();
        new VehicleRequestPacket(world, minX, minZ, maxX, maxZ).sendToServer();
    }

    @Override
    public void draw(IScreenBuilder builder, RenderState state) {
        int width  = GUIHelpers.getScreenWidth();
        int height = GUIHelpers.getScreenHeight();

        GUIHelpers.drawRect(0, 0, width, height, 0xFF14141E);

        RenderState mapState = state.clone().translate(width / 2.0, height / 2.0, 0);

        drawGrid(mapState, width, height);
        if (data != null) {
            drawRegionsAndTracks(mapState, width, height);
            drawVehicles(mapState, width, height);
        }
        drawPlayerMarker(mapState, width, height);
        drawHud(width, height);

        // 绘制道岔 UI 按钮
        drawSwitchUI(width, height);
    }

    private void drawSwitchUI(int width, int height) {
        if (selectedSwitch == null) return;

        Vec3i pos = selectedSwitch.getBlockPos();
        int sx = (int) (width / 2.0 + (pos.x - panX) * zoom);
        int sy = (int) (height / 2.0 + (pos.z - panZ) * zoom);

        // 按钮布局：第一个按钮在 sx+15，第二个在第一个右边+4，关闭按钮在第二个右边+4
        int btnY = sy - 15;
        int btnW = 70; // 保持第一个按钮的宽度
        int btnH = 16;
        int gap = 4;
        int closeW = 16;

        int btn1X = sx + 15;
        int btn2X = btn1X + btnW + gap;
        int closeX = btn2X + btnW + gap;

        Matrix4 m = new Matrix4();
        String currentStateStr = selectedSwitch.getSwitchStateEnum().toString();

        // 1. 第一个按钮 (Offline / 内存切换)
        boolean hoverBtn1 = lastMouseX >= btn1X && lastMouseX <= btn1X + btnW && lastMouseY >= btnY && lastMouseY <= btnY + btnH;
        GUIHelpers.drawRect(btn1X, btnY, btnW, btnH, hoverBtn1 ? COLOR_BTN_HOVER : COLOR_BTN_BG);
        String text1 = GuiText.GLOBAL_TRACK_MAP_SWITCHSTATE_OFFLINE.toString(currentStateStr);
        GUIHelpers.drawCenteredString(text1, btn1X + btnW / 2, btnY + 4, COLOR_BTN_TEXT, m);

        // 2. 第二个按钮 (ByBlock / 实体强制切换)
        boolean hoverBtn2 = lastMouseX >= btn2X && lastMouseX <= btn2X + btnW && lastMouseY >= btnY && lastMouseY <= btnY + btnH;
        GUIHelpers.drawRect(btn2X, btnY, btnW, btnH, hoverBtn2 ? COLOR_BTN_HOVER : COLOR_BTN_BG);
        String text2 = GuiText.GLOBAL_TRACK_MAP_SWITCHSTATE_BYBLOCK.toString(currentStateStr);
        GUIHelpers.drawCenteredString(text2, btn2X + btnW / 2, btnY + 4, COLOR_BTN_TEXT, m);

        // 3. 关闭按钮
        boolean hoverClose = lastMouseX >= closeX && lastMouseX <= closeX + closeW && lastMouseY >= btnY && lastMouseY <= btnY + btnH;
        GUIHelpers.drawRect(closeX, btnY, closeW, btnH, hoverClose ? COLOR_BTN_CLOSE_HOVER : COLOR_BTN_CLOSE);
        GUIHelpers.drawCenteredString("X", closeX + closeW / 2, btnY + 4, COLOR_BTN_TEXT, m);
    }

    private void drawVehicles(RenderState state, int width, int height) {
        if (data.clientMapVehicles == null || data.clientMapVehicles.isEmpty()) return;
        FlatCurveRenderer fr = new FlatCurveRenderer(state, null);
        double yScale = -zoom;

        for (Vec3d pos : data.clientMapVehicles) {
            double dx = pos.x - panX;
            double dz = pos.z - panZ;
            if (Math.abs(dx) > (width / 2.0) / zoom + 10 || Math.abs(dz) > (height / 2.0) / zoom + 10) continue;

            double size = 4.0;
            double w = size / zoom;
            double h = -size / zoom;

            Vec3d top = new Vec3d(dx, dz + h, 0), right = new Vec3d(dx + w, dz, 0);
            Vec3d bottom = new Vec3d(dx, dz - h, 0), left = new Vec3d(dx - w, dz, 0);

            drawExtendedLine(fr, top, right, 1.0, zoom, yScale, COLOR_VEHICLE);
            drawExtendedLine(fr, right, bottom, 1.0, zoom, yScale, COLOR_VEHICLE);
            drawExtendedLine(fr, bottom, left, 1.0, zoom, yScale, COLOR_VEHICLE);
            drawExtendedLine(fr, left, top, 1.0, zoom, yScale, COLOR_VEHICLE);
        }
    }

    public boolean handleMouse(ClientEvents.MouseGuiEvent evt) {
        int width  = GUIHelpers.getScreenWidth();
        int height = GUIHelpers.getScreenHeight();
        double cx = width / 2.0;
        double cy = height / 2.0;

        // 更新鼠标缓存坐标
        lastMouseX = evt.x;
        lastMouseY = evt.y;

        switch (evt.action) {
            case SCROLL: {
                double worldXBefore = panX + (evt.x - cx) / zoom;
                double worldZBefore = panZ + (evt.y - cy) / zoom;
                zoom = clamp(zoom * Math.pow(1.15, evt.scroll), 0.1, 64.0);
                panX = worldXBefore - (evt.x - cx) / zoom;
                panZ = worldZBefore - (evt.y - cy) / zoom;
                return false;
            }
            case CLICK: {
                boolean isLeftClick = (evt.button == 0);

                // 优先处理道岔 UI 按钮点击
                if (selectedSwitch != null && isLeftClick) {
                    // 关闭按钮
                    if (isMouseInCloseBtn(evt.x, evt.y)) {
                        selectedSwitch = null;
                        return false;
                    }

                    // 第一个按钮：内存切换 (online = false)
                    if (isMouseInSwitchBtn(evt.x, evt.y)) {
                        SwitchState target = selectedSwitch.circleSwitchState();
                        selectedSwitch.updateSwitchSate(target); // 客户端立即更新
                        new SwitchChangePacket(world, selectedSwitch.getBlockPos(), target, false).sendToServer();
                        return false;
                    }

                    // 第二个按钮：实体强制切换 (online = true)
                    if (isMouseInSyncBtn(evt.x, evt.y)) {
                        SwitchState target = selectedSwitch.circleSwitchState();
                        selectedSwitch.updateSwitchSate(target);
                        new SwitchChangePacket(world, selectedSwitch.getBlockPos(), target, true).sendToServer();
                        return false;
                    }
                }

                // 2. 处理点击选中道岔
                if (hoveredSwitch != null && isLeftClick) {
                    selectedSwitch = hoveredSwitch;
                    return false;
                }

                // 3. 否则进入地图拖拽
                if (isLeftClick) {
                    selectedSwitch = null;
                    dragging = true;
                    dragStartScreenX = evt.x; dragStartScreenY = evt.y;
                    dragStartPanX = panX; dragStartPanZ = panZ;
                }
                return false;
            }
            case MOVE: {
                if (dragging) {
                    panX = dragStartPanX - (evt.x - dragStartScreenX) / zoom;
                    panZ = dragStartPanZ - (evt.y - dragStartScreenY) / zoom;
                    return false;
                }

                // 悬停探测
                double mouseWorldX = panX + (evt.x - cx) / zoom;
                double mouseWorldZ = panZ + (evt.y - cy) / zoom;

                // 屏幕 15 像素对应的世界距离半径（平方）
                double radiusWorld = 15.0 / zoom;
                double radiusSq = radiusWorld * radiusWorld;

                hoveredSwitch = null;
                double closestDistSq = Double.MAX_VALUE;

                if (data != null) {
                    // 1. 算出鼠标所在位置的 Region ID
                    long regionId = TrackRegionUtil.vecToRegion(new Vec3i((int)Math.floor(mouseWorldX), 0, (int)Math.floor(mouseWorldZ)));

                    // 2. 直接获取该 Region 的占位方块索引
                    Map<Vec3i, List<Vec3i>> regionMap = data.trackBlockParentsByRegion.get(regionId);

                    if (regionMap != null) {
                        // 3. 遍历该 Region 里的所有占位方块
                        for (Map.Entry<Vec3i, List<Vec3i>> entry : regionMap.entrySet()) {
                            Vec3i blockPos = entry.getKey();

                            // 计算鼠标距离该方块中心的平面距离 (XZ)
                            double dx = blockPos.x + 0.5 - mouseWorldX;
                            double dz = blockPos.z + 0.5 - mouseWorldZ;
                            double d2 = dx * dx + dz * dz;

                            // 4. 如果在鼠标附近且比之前的更近
                            if (d2 < radiusSq && d2 < closestDistSq) {
                                // 获取占用该方块的所有父级轨道
                                for (Vec3i parentAbs : entry.getValue()) {
                                    TrackMultiGeometrySegment seg = data.getTrackBlock(parentAbs);
                                    // 只要父级是道岔，就命中
                                    if (seg != null && seg.type == TrackItems.SWITCH) {
                                        closestDistSq = d2;
                                        hoveredSwitch = seg;
                                    }
                                }
                            }
                        }
                    }
                }
                return true;
            }
            case RELEASE: {
                if (dragging) { dragging = false; return false; }
                return true;
            }
        }
        return true;
    }

    private boolean isMouseInSwitchBtn(int mx, int my) {
        if (selectedSwitch == null) return false;
        int width = GUIHelpers.getScreenWidth(), height = GUIHelpers.getScreenHeight();
        Vec3i pos = selectedSwitch.getBlockPos();
        int sx = (int) (width / 2.0 + (pos.x - panX) * zoom);
        int sy = (int) (height / 2.0 + (pos.z - panZ) * zoom);
        int btnX = sx + 15, btnY = sy - 15;
        return mx >= btnX && mx <= btnX + 70 && my >= btnY && my <= btnY + 16;
    }

    private boolean isMouseInSyncBtn(int mx, int my) {
        if (selectedSwitch == null) return false;
        int width = GUIHelpers.getScreenWidth(), height = GUIHelpers.getScreenHeight();
        Vec3i pos = selectedSwitch.getBlockPos();
        int sx = (int) (width / 2.0 + (pos.x - panX) * zoom);
        int sy = (int) (height / 2.0 + (pos.z - panZ) * zoom);
        int btn2X = sx + 15 + 70 + 4;
        int btnY = sy - 15;
        return mx >= btn2X && mx <= btn2X + 70 && my >= btnY && my <= btnY + 16;
    }

    private boolean isMouseInCloseBtn(int mx, int my) {
        if (selectedSwitch == null) return false;
        int width = GUIHelpers.getScreenWidth(), height = GUIHelpers.getScreenHeight();
        Vec3i pos = selectedSwitch.getBlockPos();
        int sx = (int) (width / 2.0 + (pos.x - panX) * zoom);
        int sy = (int) (height / 2.0 + (pos.z - panZ) * zoom);
        int closeX = sx + 15 + 70 + 4 + 70 + 4;
        int btnY = sy - 15;
        return mx >= closeX && mx <= closeX + 16 && my >= btnY && my <= btnY + 16;
    }

    private void collectDiagnostics() {
        diagRegionCount = 0; diagTrackCount = 0; diagPointCount = 0; diagNullCacheCount = 0;
        diagTrackMinX = Double.MAX_VALUE; diagTrackMinZ = Double.MAX_VALUE;
        diagTrackMaxX = -Double.MAX_VALUE; diagTrackMaxZ = -Double.MAX_VALUE;

        if (data == null) return;
        diagRegionCount = data.regions.size();
        for (TrackRegion region : data.regions.values()) {
            for (TrackMultiGeometrySegment seg : region.trackBlocks.values()) {
                for (Map<Gauge, TrackSingleGeometrySegment> branch : seg.paths) {
                    for (TrackSingleGeometrySegment single : branch.values()) {
                        diagTrackCount++;
                        Vec3i bp = single.getBlockPos();
                        diagTrackMinX = Math.min(diagTrackMinX, bp.x); diagTrackMaxX = Math.max(diagTrackMaxX, bp.x);
                        diagTrackMinZ = Math.min(diagTrackMinZ, bp.z); diagTrackMaxZ = Math.max(diagTrackMaxZ, bp.z);
                        List<VecYPR> points = single.pointsCache;
                        if (points == null || points.size() < 2) diagNullCacheCount++;
                        else diagPointCount += points.size();
                    }
                }
            }
        }
    }

    private void drawGrid(RenderState state, int width, int height) {
        FlatCurveRenderer fr = new FlatCurveRenderer(state, null);
        double halfW = (width / 2.0) / zoom;
        double halfH = (height / 2.0) / zoom;
        double minX = panX - halfW, maxX = panX + halfW;
        double minZ = panZ - halfH, maxZ = panZ + halfH;

        double[] spacings = {1, 4, 16, 64, 256, 1024};
        double spacing = spacings[spacings.length - 1];
        for (double s : spacings) { if (s * zoom >= 24) { spacing = s; break; } }
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

        for (Map.Entry<Long, TrackRegion> entry : data.regions.entrySet()) {
            long regionId = entry.getKey();
            int rx = TrackRegionUtil.regionX(regionId);
            int rz = TrackRegionUtil.regionZ(regionId);
            int rMinX = rx << 9; int rMinZ = rz << 9;
            int rMaxX = rMinX + 512; int rMaxZ = rMinZ + 512;
            if (rMaxX < minX || rMinX > maxX || rMaxZ < minZ || rMinZ > maxZ) continue;
            fr.drawLine(new Vec3d(rMinX - panX, rMinZ - panZ, 0), new Vec3d(rMaxX - panX, rMinZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
            fr.drawLine(new Vec3d(rMaxX - panX, rMinZ - panZ, 0), new Vec3d(rMaxX - panX, rMaxZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
            fr.drawLine(new Vec3d(rMaxX - panX, rMaxZ - panZ, 0), new Vec3d(rMinX - panX, rMaxZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
            fr.drawLine(new Vec3d(rMinX - panX, rMaxZ - panZ, 0), new Vec3d(rMinX - panX, rMinZ - panZ, 0), COLOR_REGION, zoom, yScale, 1.0);
        }

        for (TrackRegion region : data.regions.values()) {
            for (TrackMultiGeometrySegment segment : region.trackBlocks.values()) {
                Color currentColor = COLOR_TRACK;
                if (segment == hoveredSwitch) {
                    currentColor = COLOR_SWITCH_HOVER;
                } else if (segment == selectedSwitch) {
                    currentColor = COLOR_SWITCH_SELECT;
                }

                for (Map<Gauge, TrackSingleGeometrySegment> branch : segment.paths) {
                    for (TrackSingleGeometrySegment single : branch.values()) {
                        drawSingleSegment(fr, single, panX, panZ, minX, maxX, minZ, maxZ, yScale, currentColor);
                    }
                }
            }
        }
    }

    private void drawSingleSegment(FlatCurveRenderer fr, TrackSingleGeometrySegment single,
                                   double panX, double panZ,
                                   double minX, double maxX, double minZ, double maxZ, double yScale, Color color) {
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
                VecYPR a = points.get(i); VecYPR b = points.get(i + 1);
                double ax = a.x + ox, az = a.z + oz; double bx = b.x + ox, bz = b.z + oz;
                if (!segmentVisible(ax, az, bx, bz, minX, maxX, minZ, maxZ)) continue;
                fr.drawLine(new Vec3d(ax - panX, az - panZ, 0), new Vec3d(bx - panX, bz - panZ, 0), color, zoom, yScale, 1);
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
                double ax = prev.x + ox, az = prev.z + oz; double bx = curr.x + ox, bz = curr.z + oz;
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
        double dx = p.x - panX, dy = p.z - panZ;
        double size = 4, lineWidth = 1;
        double w = size / zoom, h = -size / zoom, yScale = -zoom;

        Vec3d top = new Vec3d(dx, dy + h, 0), right = new Vec3d(dx + w, dy, 0);
        Vec3d bottom = new Vec3d(dx, dy - h, 0), left = new Vec3d(dx - w, dy, 0);

        FlatCurveRenderer fr = new FlatCurveRenderer(state, null);
        drawExtendedLine(fr, top, right, lineWidth, zoom, yScale, COLOR_PLAYER);
        drawExtendedLine(fr, right, bottom, lineWidth, zoom, yScale, COLOR_PLAYER);
        drawExtendedLine(fr, bottom, left, lineWidth, zoom, yScale, COLOR_PLAYER);
        drawExtendedLine(fr, left, top, lineWidth, zoom, yScale, COLOR_PLAYER);
    }

    private void drawExtendedLine(FlatCurveRenderer fr, Vec3d start, Vec3d end, double lineWidth, double xScale, double yScale, Color color) {
        Vec3d dir = end.subtract(start).normalize();
        double extLen = (lineWidth / 2.0) / Math.abs(xScale);
        fr.drawLine(start.subtract(dir.scale(extLen)), end.add(dir.scale(extLen)), color, xScale, yScale, lineWidth);
    }

    private void drawHud(int width, int height) {
        Vec3d p = MinecraftClient.getPlayer().getPosition();
        Matrix4 m = new Matrix4();
        int y = 10;
        GUIHelpers.drawCenteredString(String.format("Zoom: %.2fx", zoom), width / 2, y, 0xFFFFFFFF, m); y += 14;
        GUIHelpers.drawCenteredString(String.format("Center: %d, %d   Player: %d, %d", (int) panX, (int) panZ, (int) p.x, (int) p.z), width / 2, y, 0xFFFFFFFF, m); y += 14;
        GUIHelpers.drawCenteredString(String.format("Regions=%d  Tracks=%d  Points=%d  NullCache=%d", diagRegionCount, diagTrackCount, diagPointCount, diagNullCacheCount), width / 2, y, 0xFFFFFF80, m); y += 14;
        if (diagTrackCount > 0) {
            GUIHelpers.drawCenteredString(String.format("TrackBBox: X[%d,%d] Z[%d,%d]", (int) diagTrackMinX, (int) diagTrackMaxX, (int) diagTrackMinZ, (int) diagTrackMaxZ), width / 2, y, 0xFFFFFF80, m);
        } else {
            GUIHelpers.drawCenteredString("No track data on client yet", width / 2, y, 0xFFFF6060, m);
        }
        GUIHelpers.drawCenteredString("Scroll = zoom   Drag = pan   ESC = close", width / 2, height - 16, 0xFFA0A0A0, m);
    }

    private boolean segmentVisible(double wx1, double wz1, double wx2, double wz2, double minX, double maxX, double minZ, double maxZ) {
        double sMinX = Math.min(wx1, wx2), sMaxX = Math.max(wx1, wx2);
        double sMinZ = Math.min(wz1, wz2), sMaxZ = Math.max(wz1, wz2);
        return !(sMaxX < minX || sMinX > maxX || sMaxZ < minZ || sMinZ > maxZ);
    }

    private static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}