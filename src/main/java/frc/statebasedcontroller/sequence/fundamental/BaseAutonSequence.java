package frc.statebasedcontroller.sequence.fundamental;

import java.util.Arrays;
import java.util.List;

import frc.pathplanner.PathPlannerFollower;
import frc.statebasedcontroller.subsystem.general.swervedrive.BaseDriveSubsystem;

public abstract class BaseAutonSequence<S extends IAutonPhase> extends BaseSequence<S> {

    final BaseDriveSubsystem baseDriveSubsystem;
    PathPlannerFollower pathPlannerFollower;
    List<PathPlannerFollower> paths;

    public BaseAutonSequence(S neutralState, S startState, BaseDriveSubsystem driveSubsystem) {
        super(neutralState, startState);
        this.baseDriveSubsystem = driveSubsystem;
    }

    public void setPathPlannerFollowers(PathPlannerFollower... pathPlannerFollowers) {
        this.paths = Arrays.asList(pathPlannerFollowers);
    }

    public PathPlannerFollower getPlannerFollower() {
        return pathPlannerFollower;
    }

    protected void setPathPlannerFollowerAtStartOfState(boolean setInitialPositionAndHeading) {
        if (this.getPhaseFirstRunThrough()) {
            this.pathPlannerFollower = this.getPhase().getPath(this);
            this.pathPlannerFollower.resetStart();
            this.getBaseDriveSubsystem().setPathPlannerFollower(getPlannerFollower(), setInitialPositionAndHeading);
        }
    }

    protected void setInitialPoseFromCurrentPath() {
        if (this.getPhaseFirstRunThrough()) {
            this.getBaseDriveSubsystem().setInitalPoseFromFirstPathPlannerFollower(this.getPhase().getPath(this));
        }
    }

    protected BaseDriveSubsystem getBaseDriveSubsystem() {
        return baseDriveSubsystem;
    }

    public List<PathPlannerFollower> getPaths() {
        return paths;
    }
}