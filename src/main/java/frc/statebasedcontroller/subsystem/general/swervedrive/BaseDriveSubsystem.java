package frc.statebasedcontroller.subsystem.general.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.pathplanner.PathPlannerFollower;
import frc.pathplanner.config.PathPlannerConfig;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;
import frc.statebasedcontroller.subsystem.general.swervedrive.swervelib.SwerveDrivetrainModel;
import frc.statebasedcontroller.subsystem.general.swervedrive.swervelib.SwerveInput;
import java.util.concurrent.Callable;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/**
 * An extension of {@link BaseSubsystem}, the base drive subsystem is a
 * subsystem for swerve drive robots. It accomodates for the additional
 * functionality required by the control system of the swerve drive modules as a
 * whole. It can track the pose of the robot, follow a path autonomously, and
 * control the robot manually.
 */
public abstract class BaseDriveSubsystem<SsS extends ISubsystemState> extends BaseSubsystem<SsS> {
    static Rotation2d targetShootRotationAngle = new Rotation2d();
    final SwerveDrivetrainModel dt;
    final SwerveDriveKinematics kinematics;
    PathPlannerFollower pathPlannerFollower;

    /**
     * @param dt           the drivetrain model that encompasses the modules and
     *                     allows for tracking and control
     * @param kinematics   the physical locations of the swerve modules relative
     *                     to the center of the robot
     * @param neutralState the state, typically labeled NEUTRAL, where no calls
     *                     are made to components unless to turn them off.
     */
    public BaseDriveSubsystem(SwerveDrivetrainModel dt,
                              SwerveDriveKinematics kinematics,
                              SsS neutralState) {
        super(neutralState);
        this.dt = dt;
        this.kinematics = kinematics;
    }

    /**
     * called every loop of the robot code. checks if the subsystem is still
     * required followed by calling the lamda expression for the current state
     * of the subsystem which creates the functionality of the robot
     * additionally, the drive subsystem keeps track of telemetry, send the
     * desired states of the modules to the modules and updates the simulation
     * if in simulation mode
     */
    public void process() {
        dt.updateOdometry();
        super.process();
    }

    /**
     * @return the path currently being followed by the drive subsystem during
     *         autonomous
     */
    public PathPlannerFollower getPathPlannerFollower() {
        return pathPlannerFollower;
    }

    /**
     * Set the desired {@link PathPlannerFollower} to follow during autonomous
     * 
     * @param ppf                the desired path follower to follow during
     *                           phase of autonomous sequence
     * @param setInitialPosition should the pose of the robot be reset to the
     *                           initial pose of the path
     */
    public void setPathPlannerFollower(PathPlannerFollower ppf,
                                       boolean setInitialPosition) {
        this.pathPlannerFollower = ppf;
        if (setInitialPosition) {
            dt.setKnownState(ppf.getInitialState());
        }
    }

    /**
     * @return get velocity vector of the robot with respect to the robot in
     *         meters per second (m/s)
     */
    public Translation2d getRobotCentricVelocity() {
        ChassisSpeeds chassisSpeeds
                        = kinematics.toChassisSpeeds(dt.getSwerveModuleStates());
        return new Translation2d(chassisSpeeds.vxMetersPerSecond,
                                 chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * @return get velocity vecotr of the robot with respect to the target in
     *         meters per second (m/s)
     */
    public Translation2d getTargetOrientedVelocity() {
        Translation2d robotCentricVelocity = getRobotCentricVelocity();
        robotCentricVelocity.rotateBy(Rotation2d.fromDegrees(180.0));
        return robotCentricVelocity;
    }

    /**
     * @return the rotational error to aim the robot at the target
     */
    public Rotation2d getTargetShootRotationAngleError() {
        return targetShootRotationAngle.minus(dt.getPose().getRotation());
    }

    /**
     * Set the rotation the robot should turn to in order to aim the front at
     * the target
     * 
     * @param angleErrorFunction typically using vision, the function that
     *                           returns the offset to aim at target
     */
    public void setTargetShootRotationAngle(Callable<Rotation2d> angleErrorFunction) {
        try {
            targetShootRotationAngle
                            = dt.getPose().getRotation().plus(angleErrorFunction.call());
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    /**
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return dt.getPose();
    }

    /**
     * @return the current gyroscope rotation
     */
    public Rotation2d getGyroRotation() {
        return dt.getGyroHeading();
    }

    /**
     * reset the gyroscope rotation to zero to indicate the front is pointing
     * straight ahead
     */
    public void resetGyro() {
        dt.zeroGyroscope(new Rotation2d());
    }

    /**
     * @return the current states of the swerve modules from the drivetrain
     *         model
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return dt.getSwerveModuleStates();
    }

    /**
     * Set the desired module states based on desired chassis speeds
     * 
     * @param chassisSpeeds the speed in each resprect that the robot that is
     *                      currently desired
     */
    protected void setModuleStates(ChassisSpeeds chassisSpeeds) {
        dt.setModuleStates(chassisSpeeds);
    }

    /**
     * Set the desired module states based on proportional inputs, typically
     * from a controller or similar device
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction
     *            to drive the robot
     * @param y   the proportion (-1 to 1) in the left and right direction to
     *            drive the robot
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot
     */
    protected void setModuleStates(double x, double y, double rot) {
        dt.setModuleStates(new SwerveInput(x, y, rot), false);
    }

    /**
     * Set the desired module states based on proportional inputs, typically
     * from a controller or similar device, but with creep proportions
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction
     *            to drive the robot cut to creep proportion
     * @param y   the proportion (-1 to 1) in the left and right direction to
     *            drive the robot cut to creep proportion
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot cut to creep proportion
     */
    protected void setModuleStatesCreep(double x, double y, double rot) {
        dt.setModuleStates(new SwerveInput(x, y, rot), true);
    }

    /**
     * Based on the current state of the path follower and the current pose, the
     * holonomic controller of the drivetrain model will calculate the desired
     * module states
     */
    public void setModuleStatesAutonomous() {
        PathPlannerState currState
                        = this.getPathPlannerFollower().getCurrentState();
        if (currState.poseMeters.getTranslation().getDistance(getPose().getTranslation()) > 0.150
            && PathPlannerConfig.USE_DIST_CORRECTION_MODE) {
            this.getPathPlannerFollower().shiftTimeToClosestPoint(getPose());
            currState = this.getPathPlannerFollower().getCurrentState();
        }
        dt.goToPose(currState);
    }
}
