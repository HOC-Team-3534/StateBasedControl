package frc.statebasedcontroller.subsystem.general.swervedrive.swervelib;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrainModel {
    public final static int NUM_MODULES = 4;
    final SwerveModule[] modules = new SwerveModule[NUM_MODULES];
    final Gyro gyro;
    final SwerveDrivePoseEstimator poseEstimator;
    private static final SendableChooser<String> orientationChooser
                    = new SendableChooser<>();
    final HolonomicDriveController holo;
    Rotation2d simGyroAngleCache = new Rotation2d();

    public SwerveDrivetrainModel(SwerveModule frontLeftModule,
                                 SwerveModule frontRightModule,
                                 SwerveModule backLeftModule,
                                 SwerveModule backRighModule, Gyro gyro) {
        modules[0] = frontLeftModule;
        modules[1] = frontRightModule;
        modules[2] = backLeftModule;
        modules[3] = backRighModule;
        this.gyro = gyro;
        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry
         * readings. The numbers used below are robot specific, and should be
         * tuned.
         */
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics,
                                                     getRawGyroHeading(),
                                                     getModulePositions(),
                                                     new Pose2d(),
                                                     VecBuilder.fill(SwerveConstants.modulePoseEstXStdDev, SwerveConstants.modulePoseEstYStdDev, SwerveConstants.modulePoseEstAngleStdDev.getRadians()),
                                                     VecBuilder.fill(SwerveConstants.visionPoseEstXStdDev, SwerveConstants.visionPoseEstYStdDev, SwerveConstants.visionPoseEstAngleStdDev.getRadians()));
        holo = new HolonomicDriveController(new PIDController(SwerveConstants.autonDriveKP,
                                                              0, 0),
                                            new PIDController(SwerveConstants.autonDriveKP,
                                                              0, 0),
                                            new ProfiledPIDController(SwerveConstants.autonSteerKP,
                                                                      0, 0,
                                                                      new TrapezoidProfile.Constraints(SwerveConstants.robotMaxAngularVel,
                                                                                                       SwerveConstants.robotMaxAngularAccel)));
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        poseEstimator.update(getRawGyroHeading(), getModulePositions());
    }

    public void updateOdometryWithVision(Pose2d botPose, double latency) {
        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp() - latency); // TODO determine latency of camera based on timestamps or other
    }

    public void setModuleStates(SwerveInput input, boolean creep,
                                boolean isOpenLoop) {
        var driveProp = creep ? SwerveConstants.slowDriveProp
                              : SwerveConstants.fastDriveProp;
        var steerProp = creep ? SwerveConstants.slowSteerProp
                              : SwerveConstants.fastSteerProp;
        var modMaxSpeed = driveProp * SwerveConstants.maxSpeed;
        var modMaxAngularSpeed = steerProp * SwerveConstants.robotMaxAngularVel;
        input = handleStationary(input);
        switch (orientationChooser.getSelected()) {
            case "Field Oriented":
                setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(input.m_translationX * modMaxSpeed, input.m_translationY * modMaxSpeed, input.m_rotation * modMaxAngularSpeed, getGyroHeading()), isOpenLoop);
                break;

            case "Robot Oriented":
                setModuleStates(new ChassisSpeeds(input.m_translationX * modMaxSpeed,
                                                  input.m_translationY * modMaxSpeed,
                                                  input.m_rotation * modMaxAngularSpeed), isOpenLoop);
                break;
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds,
                                boolean isOpenLoop) {
        simGyroAngleCache
                        = simGyroAngleCache.plus(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.020));
        SwerveModuleState[] swerveModuleStates
                        = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);
        for (int i = 0; i < NUM_MODULES; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    public void setVoltageToZero() {
        for (int i = 0; i < NUM_MODULES; i++) {
            modules[i].setDriveVoltageForCharacterization(0);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions
                        = new SwerveModulePosition[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[NUM_MODULES];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setKnownPose(Pose2d in) {
        poseEstimator.resetPosition(getRawGyroHeading(), getModulePositions(), in);
    }

    public void setKnownState(PathPlannerState initialState) {
        Pose2d startingPose
                        = new Pose2d(initialState.poseMeters.getTranslation(),
                                     initialState.holonomicRotation);
        setKnownPose(startingPose);
    }

    private Rotation2d getRawGyroHeading() {
        if (RobotBase.isSimulation())
            return simGyroAngleCache;
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void goToPose(PathPlannerState state) {
        setModuleStates(holo.calculate(getPose(), state.poseMeters, state.velocityMetersPerSecond, state.holonomicRotation), false);
    }

    private SwerveInput handleStationary(SwerveInput input) {
        if (input.m_rotation == 0 && input.m_translationX == 0
            && input.m_translationY == 0) {
            // Hopefully this will turn all of the modules to the "turning" configuration so
            // being pushed is more difficult
            input.m_rotation = 0.0; // 001;
        }
        return input;
    }

    public void setModuleStates(SwerveInput input, Rotation2d desiredRotation,
                                boolean creep, boolean resetController) {
        var driveProp = creep ? SwerveConstants.slowDriveProp
                              : SwerveConstants.fastDriveProp;
        var modMaxSpeed = driveProp * SwerveConstants.maxSpeed;
        input = handleStationary(input);
        if (resetController)
            holo.getThetaController().reset(getGyroHeading().getRadians());
        var angularSpeed = holo.getThetaController().calculate(getGyroHeading().getRadians(), desiredRotation.getRadians());
        setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(input.m_translationX * modMaxSpeed, input.m_translationY * modMaxSpeed, angularSpeed, getGyroHeading()), false);
    }

    public SwerveModule[] getSwerveModules() {
        return modules;
    }
}
