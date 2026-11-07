package cam72cam.immersiverailroading.track;

import cam72cam.immersiverailroading.library.TrackItems;
import cam72cam.immersiverailroading.tile.TileRail;
import cam72cam.immersiverailroading.util.MultiSwitchBranch;
import cam72cam.immersiverailroading.util.RailInfo;
import cam72cam.mod.item.ItemStack;
import cam72cam.mod.math.Vec3d;
import cam72cam.mod.math.Vec3i;
import cam72cam.mod.world.World;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class BuilderMultiSwitch extends BuilderBase implements IIterableTrack {

    private List<BuilderCubicCurve> subBranchBuilders = new ArrayList<>();
    private BuilderCubicCurve mainBranchBuilder;

    public BuilderMultiSwitch(RailInfo info, World world, Vec3i pos) {
        super(info, world, pos);

        RailInfo mainInfo = info.with(mutable -> {
            mutable.multiSwitchInfo = null;
            mutable.switchBranchIndex = 1;
        });

        mainBranchBuilder = (BuilderCubicCurve) mainInfo.getBuilder(world, pos);
        mainBranchBuilder.info = mainBranchBuilder.info.with(mutable -> mutable.multiSwitchInfo = info.multiSwitchInfo);

        HashSet<Vec3i> pubic = mainBranchBuilder.positions; // todo

        for(int i = 0; i < info.multiSwitchInfo.multiSwitchBranches().size(); i++) {
            MultiSwitchBranch branch = info.multiSwitchInfo.multiSwitchBranches().get(i);
            RailInfo branchInfo = new RailInfo(branch.settings(), info.placementInfo, branch.customInfo(), null);
            int finalI = i;
            branchInfo = branchInfo.with(mutable -> mutable.switchBranchIndex = finalI + 2);
            BuilderCubicCurve branchBuilder = (BuilderCubicCurve) branchInfo.getBuilder(world, pos);

            for(TrackBase turn : branchBuilder.tracks) {
                if (turn instanceof TrackRail) {
                    turn.overrideParent(mainBranchBuilder.getParentPos());
                }
            }

            branchBuilder.overrideFlexible = true;

            subBranchBuilders.add(branchBuilder);
        }

        for (TrackBase straight : mainBranchBuilder.tracks) {
            if (straight instanceof TrackGag) {
                straight.setFlexible();
            }
        }
    }

    @Override
    public List<BuilderBase> getSubBuilders() {
        List<BuilderBase> subTurns = null;
        for(BuilderCubicCurve builder : subBranchBuilders) {
            if(subTurns == null) subTurns = new ArrayList<>();
            subTurns.addAll(builder.getSubBuilders());
        }

        List<BuilderBase> subStraights = mainBranchBuilder.getSubBuilders();

        if (subTurns == null && subStraights == null) {
            return null;
        }

        List<BuilderBase> res = new ArrayList<>();
        if (subTurns == null) {
            res.addAll(subBranchBuilders);
        } else {
            res.addAll(subTurns);
        }
        if (subStraights == null) {
            res.add(mainBranchBuilder);
        } else {
            res.addAll(subStraights);
        }
        return res;
    }

    @Override
    public int costTies() {
        int costTiles = mainBranchBuilder.costTies();
        for(BuilderCubicCurve builder: subBranchBuilders) costTiles += builder.costTies();
        return costTiles;
    }

    @Override
    public int costRails() {
        int costRails = mainBranchBuilder.costRails();
        for(BuilderCubicCurve builder: subBranchBuilders) costRails += builder.costTies();
        return costRails;
    }

    @Override
    public int costBed() {
        int costBed = mainBranchBuilder.costBed();
        for(BuilderCubicCurve builder: subBranchBuilders) costBed += builder.costTies();
        return costBed;
    }

    @Override
    public int costFill() {
        int costFill = mainBranchBuilder.costFill();
        for(BuilderCubicCurve builder: subBranchBuilders) costFill += builder.costFill();
        return costFill;
    }

    @Override
    public void setDrops(List<ItemStack> drops) {
        mainBranchBuilder.setDrops(drops);
    }


    @Override
    public boolean canBuild() {
        boolean canBuild = mainBranchBuilder.canBuild();
        for(BuilderCubicCurve builder: subBranchBuilders) {
            if (!canBuild || !builder.canBuild()) {
                canBuild = false;
                break;
            }
        }
        return canBuild;
    }

    @Override
    public void build() {
        mainBranchBuilder.info.builderBranchPosBridge = new ArrayList<>();
        mainBranchBuilder.info.builderBranchBridge = new ArrayList<>();
        for(int i = 0; i < subBranchBuilders.size(); i++) {
            BuilderCubicCurve builder = subBranchBuilders.get(i);
            mainBranchBuilder.info.builderBranchBridge.add(builder.info);
            if(builder.getSubBuilders() != null) {
                BuilderCubicCurve subBuilder = (BuilderCubicCurve) builder.getSubBuilders().getFirst();
                int finalI = i;
                subBuilder.info = subBuilder.info.with(mutable -> mutable.switchBranchIndex = finalI + 2);
                mainBranchBuilder.info.builderBranchPosBridge.add(subBuilder.getParentPos().subtract(mainBranchBuilder.getParentPos()));
            } else {
                mainBranchBuilder.info.builderBranchPosBridge.add(builder.getParentPos().subtract(mainBranchBuilder.getParentPos()));
            }
        }
        mainBranchBuilder.build();
        mainBranchBuilder.info.builderBranchBridge = null;
        mainBranchBuilder.info.builderBranchPosBridge = null;
        for(BuilderCubicCurve builder : subBranchBuilders) builder.build();
    }

    @Override
    public void clearArea() {
        mainBranchBuilder.clearArea();
        for(BuilderCubicCurve builder: subBranchBuilders) builder.clearArea();
    }

    @Override
    public List<TrackBase> getTracksForRender() {
        List<TrackBase> data = mainBranchBuilder.getTracksForRender();
        for(BuilderCubicCurve builder : subBranchBuilders) {
            data.addAll(builder.getTracksForRender());
        }
        return data;
    }

    @Override
    public List<VecYPR> getRenderData() {
        List<VecYPR> data = mainBranchBuilder.getRenderData();
        for(BuilderCubicCurve builder : subBranchBuilders) {
            data.addAll(builder.getRenderData());
        }
        return data;
    }

    @Override
    public List<VecYPR> getPath(double stepSize) {
        return mainBranchBuilder.getPath(stepSize);
    }
}