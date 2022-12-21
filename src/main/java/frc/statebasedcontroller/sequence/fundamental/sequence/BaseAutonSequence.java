package frc.statebasedcontroller.sequence.fundamental.sequence;

import java.util.Arrays;
import java.util.List;

import frc.pathplanner.PathPlannerFollower;
import frc.statebasedcontroller.sequence.fundamental.phase.ISequencePhase;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;
import frc.statebasedcontroller.subsystem.general.swervedrive.BaseDriveSubsystem;

/**
 * This base autononomous sequences class adds onto what {@link BaseSequence} has to control the robot during autonous,
 * which requires path following abilities.
 */
public abstract class BaseAutonSequence<SeqP extends ISequencePhase> extends BaseSequence<SeqP> {

    final BaseDriveSubsystem baseDriveSubsystem;
    PathPlannerFollower pathPlannerFollower;
    boolean alreadySetInitialPosition;
    List<PathPlannerFollower> paths;

    /**
     * 
     * @param neutralPhase the phase, usually enum labeled NEUTRAL, that has no required subsystems
     * @param startPhase the first phase to go to when the sequence begins running/processing
     * @param driveSubsystem the drive subsytem of the robot
     */
    public BaseAutonSequence(SeqP neutralPhase, SeqP startPhase, BaseDriveSubsystem driveSubsystem) {
        super(neutralPhase, startPhase);
        this.baseDriveSubsystem = driveSubsystem;
    }

    /**
     * When the sequence is currently in its neutral phase,
     * call this method to begin the sequence at the start phase,
     * which will require the subsystems to go to the associated state
     * 
     * @param subsystems usually empty. subsystems that must abort their current sequences, using {@link BaseSubsystem#forceRelease() forceRelease}
     */
    @Override
    public void start(BaseSubsystem... subsystems) {
        super.start(subsystems);
        if(isNeutral()){
            alreadySetInitialPosition = false;
        }
    }

    /**
     * Requires all the the subsubsystems associated with the phase and sets the phase if successful.
     * In addition to basic functionality, this also makes sure to set the path for following because this sequence is autonomous
     * 
     * @param phase the phase enum to switch the sequence to
     * @return successfully required subsystems and switch phase of sequence
     */
    boolean setPhase(SeqP phase) {
        if (phase.getPhase().requireSubsystems(this)) {
            this.phase = phase;
            resetPhaseStartTime();
            phaseFirstRunThrough = true;
            setPathPlannerFollowerAtStartOfState(!alreadySetInitialPosition);
            return true;
        }
        return false;
    }

    /**
     * On the first run through of the phase, the path follower is set and reset to the start, and then given to the drivesubsystem to use for
     * autonomous path following
     * 
     * @param setInitialPositionAndHeading whether to set the pose of the robot to the initial pose of the path, done at the first path of the auton sequence
     */
    void setPathPlannerFollowerAtStartOfState(boolean setInitialPositionAndHeading) {
        if (getPhaseFirstRunThrough()) {
            setPathPlannerFollower();
            getPlannerFollower().resetStart();
            getBaseDriveSubsystem().setPathPlannerFollower(getPlannerFollower(), setInitialPositionAndHeading);
            alreadySetInitialPosition = true;
        }
    }

    /**
     * 
     * @return the drive subsystem of the robot
     */
    protected BaseDriveSubsystem getBaseDriveSubsystem() {
        return baseDriveSubsystem;
    }

    /**
     * 
     * @return the current path being followed
     */
    public PathPlannerFollower getPlannerFollower() {
        return pathPlannerFollower;
    }

    /**
     * 
     * Set the PathPlannerFollower based on current phase
     */
    void setPathPlannerFollower(){
        int index = getPhase().getPhase().getPathIndex();
        if(index > 0 && !(index >= getPathPlannerFollowers().size())){
            this.pathPlannerFollower = getPathPlannerFollowers().get(index);
        }
    }

    /**
     * 
     * @return the list of paths for the auton
     */
    List<PathPlannerFollower> getPathPlannerFollowers(){
        return paths;
    }

    /**
     * 
     * @param paths the paths for the auton sequence to follow, corresponding with the path indeces in the sequence phases
     */
    public void setPathPlannerFollowers(List<PathPlannerFollower> paths){
        this.paths = paths;
    }

    /**
     * 
     * @param paths the paths for the auton sequence to follow, corresponding with the path indeces in the sequence phases
     */
    public void setPathPlannerFollowers(PathPlannerFollower... paths){
        this.paths = Arrays.asList(paths);
    }
}