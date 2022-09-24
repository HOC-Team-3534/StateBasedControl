package frc.statebasedcontroller.subsystem.general.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.pathplanner.PathPlannerFollower;
import frc.statebasedcontroller.config.DriveSpeedsConfig;
import frc.statebasedcontroller.subsystem.fundamental.BaseSubsystem;
import frc.statebasedcontroller.subsystem.fundamental.ISubsystemState;
import frc.statebasedcontroller.subsystem.requiredadditions.swervedrive.StateBasedSwerveDrivetrainModel;
import frc.swervelib.SwerveConstants;
import frc.swervelib.SwerveInput;
import frc.swervelib.SwerveModule;
import frc.wpiClasses.QuadSwerveSim;

import java.util.ArrayList;
import java.util.concurrent.Callable;

public abstract class BaseDriveSubsystem<SsS extends ISubsystemState> extends BaseSubsystem<SsS> implements IDriveSubsystem {
    static Rotation2d targetShootRotationAngle = new Rotation2d();
    // Locations for the swerve drive modules relative to the robot center.
    StateBasedSwerveDrivetrainModel dt;
    ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
    double frontLeft_stateAngle = 0.0,
            frontRight_stateAngle = 0.0,
            backLeft_stateAngle = 0.0,
            backRight_stateAngle = 0.0;
    PathPlannerFollower pathPlannerFollower;
    SwerveDriveKinematics kinematics;

    public BaseDriveSubsystem(StateBasedSwerveDrivetrainModel dt, SwerveDriveKinematics kinematics, SsS neutralState) {
        super(neutralState);
        this.dt = dt;
        this.kinematics = kinematics;
        modules = dt.getRealModules();
    }

    public void process() {
        dt.updateTelemetry();
        super.process();
        sendStates();
        if(RobotBase.isSimulation()){
        dt.update(DriverStation.isDisabled(), 13.2);
        }
    }

    public void setPathPlannerFollower(PathPlannerFollower ppf, boolean setInitialPosition) {
        this.pathPlannerFollower = ppf;
        if (setInitialPosition) {
            setInitalPoseFromFirstPathPlannerFollower(ppf);
        }
    }

    public PathPlannerFollower getPathPlannerFollower(){
        return pathPlannerFollower;
    }

    public void setInitalPoseFromFirstPathPlannerFollower(PathPlannerFollower ppf) {
        dt.setKnownState(ppf.getInitialState());
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return dt.getSwerveModuleStates();
    }

    public Vector2d getRobotCentricVelocity() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(dt.getSwerveModuleStates());
        return new Vector2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    public Vector2d getTargetOrientedVelocity() {
        Vector2d robotCentricVelocity = getRobotCentricVelocity();
        robotCentricVelocity.rotate(180.0);
        return robotCentricVelocity;
    }

    public void setTargetShootRotationAngle(Callable<Rotation2d> angleErrorFunction) {
        //this.targetShootRotationAngle = getGyroHeading().plus(Robot.limelight.getLimelightShootProjection().getOffset());
        try {
            targetShootRotationAngle = dt.getGyroscopeRotation().plus(angleErrorFunction.call());
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public Rotation2d getTargetShootRotationAngleError() {
        return targetShootRotationAngle.minus(dt.getGyroscopeRotation());
    }

    public Rotation2d getGyroRotation(){
        return dt.getGyroscopeRotation();
    }

    public void resetGyro(){
        dt.zeroGyroscope();
    }

    public Pose2d getPose(){
        return dt.getPose();
    }

    protected void setModuleStates(ChassisSpeeds chassisSpeeds){
        dt.setModuleStates(chassisSpeeds);
    }

    protected void setModuleStates(double x, double y, double rot){
        setFastSpeeds();
        dt.setModuleStates(new SwerveInput(x, y, rot));
    }

    protected void setModuleStatesCreep(double x, double y, double rot){
        setSlowSpeeds();
        dt.setModuleStates(new SwerveInput(x, y, rot));
    }

    protected void setModuleStatesCreepAim(double x, double y, double rot){
        setSlowSpeeds();
        dt.setModuleStates(new SwerveInput(x, y, rot));
    }

    public void setModuleStatesAutonomous() {
        dt.goToPose(this.getPathPlannerFollower().getCurrentState());
    }

    protected void setFastSpeeds(){
        dt.setMaxSpeeds(DriveSpeedsConfig.FWD_REV_FAST, DriveSpeedsConfig.STRAFE_FAST, DriveSpeedsConfig.ROTATION_FAST);
    }

    protected void setSlowSpeeds(){
        dt.setMaxSpeeds(DriveSpeedsConfig.FWD_REV_SLOW, DriveSpeedsConfig.STRAFE_SLOW, DriveSpeedsConfig.ROTATION_SLOW);
    }

    protected void sendStates() {
        SwerveModuleState[] states = dt.getSwerveModuleStates();

        if (states != null) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_FWD_REV_SPEED_MPS);

        modules.get(0).set(states[0].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[0].angle.getRadians());
        modules.get(1).set(states[1].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[1].angle.getRadians());
        modules.get(2).set(states[2].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[2].angle.getRadians());
        modules.get(3).set(states[3].speedMetersPerSecond / SwerveConstants.MAX_FWD_REV_SPEED_MPS * SwerveConstants.MAX_VOLTAGE, states[3].angle.getRadians());

        dt.getPoseEstimator().update(dt.getGyroscopeRotation(), states[0], states[1], states[2], states[3]);
        }
    }
}
