package cam72cam.immersiverailroading.track.graph;

import cam72cam.immersiverailroading.library.Gauge;
import cam72cam.immersiverailroading.tile.TileRail;
import cam72cam.immersiverailroading.track.BuilderCubicCurve;
import cam72cam.immersiverailroading.track.CubicCurve;
import cam72cam.immersiverailroading.util.RollAndOffsetInfo;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

public class TrackSingleGeometrySegment {
    protected final Gauge gauge;
    protected final int switchIdex;
    protected final CubicCurve baseCurve;
    protected final RollAndOffsetInfo rollAndOffsetInfo;
    protected final String referenceTrack;// for path piece height

    public TrackSingleGeometrySegment(TileRail trackBlock, World world, Vec3i pos, int switchIndex, Gauge gauge) {// todo: switch, gauge
        this.gauge = trackBlock.info.settings.gauge;
        this.switchIdex = switchIndex;
        this.baseCurve = new BuilderCubicCurve(trackBlock.info, world, pos, false).getCurve();
        this.rollAndOffsetInfo = trackBlock.info.settings.rollAndOffsetInfo;
        this.referenceTrack = trackBlock.info.settings.track;
    }

    public TrackSingleGeometrySegment(ByteBuffer buffer) {
        int version = buffer.getInt(); // version
        if (version != 1) {
            throw new RuntimeException(String.format("Invalid single track geometry segment data version %d", version));
        }

        switchIdex = buffer.getInt(); // switch index
        gauge = Gauge.from(buffer.getDouble()); // gauge
        referenceTrack = TrackRegionUtil.readString(buffer);// referenceTrack

        // baseCurve p1.xyz ctrl1.xyz ctrl2.xyz p2.xyz
        double[] baseCurveArgs = new double[3 * 4];
        for(int i = 0; i < 3 * 4; i++) {
            baseCurveArgs[i] = buffer.getDouble();
        }
        baseCurve = new CubicCurve(
                new Vec3d(baseCurveArgs[0], baseCurveArgs[1], baseCurveArgs[2]),
                new Vec3d(baseCurveArgs[3], baseCurveArgs[4], baseCurveArgs[5]),
                new Vec3d(baseCurveArgs[6], baseCurveArgs[7], baseCurveArgs[8]),
                new Vec3d(baseCurveArgs[9], baseCurveArgs[10], baseCurveArgs[11])
        );// todo 外部参数应该是不需要的，需要检查

        RollAndOffsetInfo.RollAndVertOffsetAlignType rollOffsetType = RollAndOffsetInfo.RollAndVertOffsetAlignType.byOrder(buffer.getInt()); // rollAndOffsetInfo.rollOffsetType
        boolean degreeMode = buffer.getShort() == 1; // rollAndOffsetInfo.degreeMode
        boolean offsetVertByNormal = buffer.getShort() == 1; // rollAndOffsetInfo.offsetVertByNormal

        int count = buffer.getInt(); // rollAndOffsetInfo.arcLenFactors count

        List<Double> arcLenFactors = new ArrayList<>();
        List<Vec3d> rolls = new ArrayList<>();
        List<Vec3d> rollCtrls = new ArrayList<>();
        List<Vec3d> yOffsets = new ArrayList<>();
        List<Vec3d> yOffsetCtrls = new ArrayList<>();
        List<Vec3d> zOffsets = new ArrayList<>();
        List<Vec3d> zOffsetCtrls = new ArrayList<>();

        for(int i = 0; i < count; i++) {
            arcLenFactors.add(buffer.getDouble()); // rollAndOffsetInfo.arcLenFactors
            double x,y,z;

            // rollAndOffsetInfo.rolls.xyz
            x = buffer.getDouble();
            y = buffer.getDouble();
            z = buffer.getDouble();
            rolls.add(new Vec3d(x, y, z));
            // rollAndOffsetInfo.rollCtrls.xyz
            x = buffer.getDouble();
            y = buffer.getDouble();
            z = buffer.getDouble();
            rollCtrls.add(new Vec3d(x, y, z));
            // rollAndOffsetInfo.yOffsets.xyz
            x = buffer.getDouble();
            y = buffer.getDouble();
            z = buffer.getDouble();
            yOffsets.add(new Vec3d(x, y, z));
            // rollAndOffsetInfo.yOffsetCtrls.xyz
            x = buffer.getDouble();
            y = buffer.getDouble();
            z = buffer.getDouble();
            yOffsetCtrls.add(new Vec3d(x, y, z));
            // rollAndOffsetInfo.zOffsets.xyz
            x = buffer.getDouble();
            y = buffer.getDouble();
            z = buffer.getDouble();
            zOffsets.add(new Vec3d(x, y, z));
            // rollAndOffsetInfo.zOffsetCtrls.xyz
            x = buffer.getDouble();
            y = buffer.getDouble();
            z = buffer.getDouble();
            zOffsetCtrls.add(new Vec3d(x, y, z));
        }

        rollAndOffsetInfo = new RollAndOffsetInfo(
                rollOffsetType, false, false, degreeMode, offsetVertByNormal,
                arcLenFactors, rolls, rollCtrls, yOffsets, yOffsetCtrls, zOffsets, zOffsetCtrls
        );
    }

    public int sizeBytes() {
        int bytes = 0;

        bytes += Integer.BYTES; // version
        bytes += Integer.BYTES; // switch index
        bytes += Double.BYTES; // gauge
        bytes += TrackRegionUtil.sizeString(referenceTrack); // referenceTrack

        // baseCurve p1.xyz ctrl1.xyz ctrl2.xyz p2.xyz
        bytes += Double.BYTES * 3 * 4;

        bytes += Integer.BYTES; // rollAndOffsetInfo.rollOffsetType
        bytes += Short.BYTES; // rollAndOffsetInfo.degreeMode
        bytes += Short.BYTES; // rollAndOffsetInfo.offsetVertByNormal

        bytes += Integer.BYTES; // rollAndOffsetInfo.arcLenFactors count
        for(int i = 0; i < rollAndOffsetInfo.arcLenFactors().size(); i++) {
            bytes += Double.BYTES; // rollAndOffsetInfo.arcLenFactors
            bytes += Double.BYTES * 3; // rollAndOffsetInfo.rolls.xyz
            bytes += Double.BYTES * 3; // rollAndOffsetInfo.rollCtrls.xyz
            bytes += Double.BYTES * 3; // rollAndOffsetInfo.yOffsets.xyz
            bytes += Double.BYTES * 3; // rollAndOffsetInfo.yOffsetCtrls.xyz
            bytes += Double.BYTES * 3; // rollAndOffsetInfo.zOffsets.xyz
            bytes += Double.BYTES * 3; // rollAndOffsetInfo.zOffsetCtrls.xyz
        }

        return bytes;
    }

    public void write(ByteBuffer buffer) {
        buffer.putInt(1); // version
        buffer.putInt(switchIdex); // switch index
        buffer.putDouble(gauge.value()); // gauge
        TrackRegionUtil.writeString(referenceTrack, buffer);// referenceTrack

        // baseCurve p1.xyz ctrl1.xyz ctrl2.xyz p2.xyz
        buffer.putDouble(baseCurve.p1.x);
        buffer.putDouble(baseCurve.p1.y);
        buffer.putDouble(baseCurve.p1.z);
        buffer.putDouble(baseCurve.ctrl1.x);
        buffer.putDouble(baseCurve.ctrl1.y);
        buffer.putDouble(baseCurve.ctrl1.z);
        buffer.putDouble(baseCurve.ctrl2.x);
        buffer.putDouble(baseCurve.ctrl2.y);
        buffer.putDouble(baseCurve.ctrl2.z);
        buffer.putDouble(baseCurve.p2.x);
        buffer.putDouble(baseCurve.p2.y);
        buffer.putDouble(baseCurve.p2.z);

        buffer.putInt(rollAndOffsetInfo.rollOffsetType().ordinal()); // rollAndOffsetInfo.rollOffsetType
        buffer.putShort(rollAndOffsetInfo.degreeMode() ? (short)1 : (short)0); // rollAndOffsetInfo.degreeMode
        buffer.putShort(rollAndOffsetInfo.offsetVertByNormal() ? (short)1 : (short)0); // rollAndOffsetInfo.offsetVertByNormal

        buffer.putInt(rollAndOffsetInfo.arcLenFactors().size()); // rollAndOffsetInfo.arcLenFactors count
        for(int i = 0; i <rollAndOffsetInfo.arcLenFactors().size(); i++) {
            buffer.putDouble(rollAndOffsetInfo.arcLenFactors().get(i)); // rollAndOffsetInfo.arcLenFactors
            // rollAndOffsetInfo.rolls.xyz
            buffer.putDouble(rollAndOffsetInfo.rolls().get(i).x);
            buffer.putDouble(rollAndOffsetInfo.rolls().get(i).y);
            buffer.putDouble(rollAndOffsetInfo.rolls().get(i).z);
            // rollAndOffsetInfo.rollCtrls.xyz
            buffer.putDouble(rollAndOffsetInfo.rollCtrls().get(i).x);
            buffer.putDouble(rollAndOffsetInfo.rollCtrls().get(i).y);
            buffer.putDouble(rollAndOffsetInfo.rollCtrls().get(i).z);
            // rollAndOffsetInfo.yOffsets.xyz
            buffer.putDouble(rollAndOffsetInfo.yOffsets().get(i).x);
            buffer.putDouble(rollAndOffsetInfo.yOffsets().get(i).y);
            buffer.putDouble(rollAndOffsetInfo.yOffsets().get(i).z);
            // rollAndOffsetInfo.yOffsetCtrls.xyz
            buffer.putDouble(rollAndOffsetInfo.yOffsetCtrls().get(i).x);
            buffer.putDouble(rollAndOffsetInfo.yOffsetCtrls().get(i).y);
            buffer.putDouble(rollAndOffsetInfo.yOffsetCtrls().get(i).z);
            // rollAndOffsetInfo.zOffsets.xyz
            buffer.putDouble(rollAndOffsetInfo.zOffsets().get(i).x);
            buffer.putDouble(rollAndOffsetInfo.zOffsets().get(i).y);
            buffer.putDouble(rollAndOffsetInfo.zOffsets().get(i).z);
            // rollAndOffsetInfo.zOffsetCtrls.xyz
            buffer.putDouble(rollAndOffsetInfo.zOffsetCtrls().get(i).x);
            buffer.putDouble(rollAndOffsetInfo.zOffsetCtrls().get(i).y);
            buffer.putDouble(rollAndOffsetInfo.zOffsetCtrls().get(i).z);
        }
    }
}
