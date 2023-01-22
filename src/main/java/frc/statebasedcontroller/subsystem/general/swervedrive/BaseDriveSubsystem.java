package frc.statebasedcontroller.subsystem.general.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.pathplanner.PathPlannerFollower;
import frc.pathplanner.config.PathPlannerConfig;
import frc.statebasedcontroller.config.DriveSpeedsConfig;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;
import frc.swervelib.SwerveConstants;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveInput;
import frc.swervelib.interfaces.SwerveModule;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.Vector2d;

import java.util.ArrayList;
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
    ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
    double frontLeft_stateAngle = 0.0, frontRight_stateAngle = 0.0, backLeft_stateAngle = 0.0,
                    backRight_stateAngle = 0.0;
    PathPlannerFollower pathPlannerFollower;
    final SwerveDriveKinematics kinematics;

    /**
     * @param dt           the drivetrain model that encompasses the modules and
     *                     allows for tracking and control
     * @param kinematics   the physical locations of the swerve modules relative to
     *                     the center of the robot
     * @param neutralState the state, typically labeled NEUTRAL, where no calls are
     *                     made to components unless to turn them off.
     */
    public BaseDriveSubsystem(SwerveDrivetrainModel dt, SwerveDriveKinematics kinematics,
                              SsS neutralState) {
        super(neutralState);
        this.dt = dt;
        this.kinematics = kinematics;
        this.modules = dt.getRealModules();
    }

    /**
     * called every loop of the robot code. checks if the subsystem is still
     * required followed by calling the lamda expression for the current state of
     * the subsystem which creates the functionality of the robot additionally, the
     * drive subsystem keeps track of telemetry, send the desired states of the
     * modules to the modules and updates the simulation if in simulation mode
     */
    public void process() {
        dt.getPoseEstimator().update(dt.getGyroscopeRotation(), dt.getPositions());
        dt.updateTelemetry();
        super.process();
        sendStates();
        if (RobotBase.isSimulation()) {
            dt.update(DriverStation.isDisabled(), 13.2);
        }
    }

    /**
     * Sends the desired module states to the swerve modules using the drivetrain
     * model. Simultaneously updates pose estimation
     */
    protected void sendStates() {
        SwerveModuleState[] states = dt.getSwerveModuleStates();
        if (states != null) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_FWD_REV_SPEED_MPS);
            modules.get(0).set(states[0].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[0].angle);
            modules.get(1).set(states[1].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[1].angle);
            modules.get(2).set(states[2].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[2].angle);
            modules.get(3).set(states[3].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[3].angle);
        }
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
     * @param ppf                the desired path follower to follow during phase of
     *                           autonomous sequence
     * @param setInitialPosition should the pose of the robot be reset to the
     *                           initial pose of the path
     */
    public void setPathPlannerFollower(PathPlannerFollower ppf, boolean setInitialPosition) {
        this.pathPlannerFollower = ppf;
        if (setInitialPosition) {
            dt.setKnownState(ppf.getInitialState());
        }
    }

    /**
     * @return get velocity vector of the robot with respect to the robot in meters
     *         per second (m/s)
     */
    public Vector2d getRobotCentricVelocity() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(dt.getSwerveModuleStates());
        return new Vector2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * @return get velocity vecotr of the robot with respect to the target in meters
     *         per second (m/s)
     */
    public Vector2d getTargetOrientedVelocity() {
        Vector2d robotCentricVelocity = getRobotCentricVelocity();
        robotCentricVelocity.rotate(180.0);
        return robotCentricVelocity;
    }

    /**
     * @return the rotational error to aim the robot at the target
     */
    public Rotation2d getTargetShootRotationAngleError() {
        return targetShootRotationAngle.minus(dt.getGyroscopeRotation());
    }

    /**
     * Set the rotation the robot should turn to in order to aim the front at the
     * target
     * 
     * @param angleErrorFunction typically using vision, the function that returns
     *                           the offset to aim at target
     */
    public void setTargetShootRotationAngle(Callable<Rotation2d> angleErrorFunction) {
        try {
            targetShootRotationAngle = dt.getGyroscopeRotation().plus(angleErrorFunction.call());
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
        return dt.getGyroscopeRotation();
    }

    /**
     * reset the gyroscope rotation to zero to indicate the front is pointing
     * straight ahead
     */
    public void resetGyro() {
        dt.zeroGyroscope();
    }

    /**
     * @return the current states of the swerve modules from the drivetrain model
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
     * Set the desired module states based on proportional inputs, typically from a
     * controller or similar device
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction to
     *            drive the robot
     * @param y   the proportion (-1 to 1) in the left and right direction to drive
     *            the robot
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot
     */
    protected void setModuleStates(double x, double y, double rot) {
        setFastSpeeds();
        dt.setModuleStates(new SwerveInput(x, y, rot));
    }

    /**
     * Set the desired module states based on proportional inputs, typically from a
     * controller or similar device, but with creep proportions
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction to
     *            drive the robot cut to creep proportion
     * @param y   the proportion (-1 to 1) in the left and right direction to drive
     *            the robot cut to creep proportion
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot cut to creep proportion
     */
    protected void setModuleStatesCreep(double x, double y, double rot) {
        setSlowSpeeds();
        dt.setModuleStates(new SwerveInput(x, y, rot));
    }

    /**
     * Set the desired module states during aiming based on proportional inputs,
     * typically from a controller or similar device, but with creep proportions
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction to
     *            drive the robot cut to creep proportion
     * @param y   the proportion (-1 to 1) in the left and right direction to drive
     *            the robot cut to creep proportion
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot cut to creep proportion
     */
    protected void setModuleStatesCreepAim(double x, double y, double rot) {
        setSlowSpeeds();
        dt.setModuleStates(new SwerveInput(x, y, rot));
    }

    /**
     * Based on the current state of the path follower and the current pose, the
     * holonomic controller of the drivetrain model will calculate the desired
     * module states
     */
    public void setModuleStatesAutonomous() {
        PathPlannerState currState = this.getPathPlannerFollower().getCurrentState();
        if (currState.poseMeters.getTranslation().getDistance(getPose().getTranslation()) > 0.150
            && PathPlannerConfig.USE_DIST_CORRECTION_MODE) {
            this.getPathPlannerFollower().shiftTimeToClosestPoint(getPose());
            currState = this.getPathPlannerFollower().getCurrentState();
        }
        dt.goToPose(currState);
    }

    /**
     * switch to fast speed
     */
    protected void setFastSpeeds() {
        dt.setMaxSpeeds(DriveSpeedsConfig.FWD_REV_FAST, DriveSpeedsConfig.STRAFE_FAST, DriveSpeedsConfig.ROTATION_FAST);
    }

    /**
     * switch to slow speed
     */
    protected void setSlowSpeeds() {
        dt.setMaxSpeeds(DriveSpeedsConfig.FWD_REV_SLOW, DriveSpeedsConfig.STRAFE_SLOW, DriveSpeedsConfig.ROTATION_SLOW);
    }
}
