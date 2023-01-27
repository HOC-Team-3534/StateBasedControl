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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrainModel {
    public final static int NUM_MODULES = 4;
    final SwerveModule[] modules = new SwerveModule[NUM_MODULES];
    final Gyro gyro;
    final SwerveDriveKinematics kinematics;
    final SwerveDrivePoseEstimator poseEstimator;
    final Configuration config;
    private static final SendableChooser<String> orientationChooser
                    = new SendableChooser<>();
    Rotation2d gyroOffset = new Rotation2d();
    final HolonomicDriveController holo;

    public SwerveDrivetrainModel(SwerveModule frontLeftModule,
                                 SwerveModule frontRightModule,
                                 SwerveModule backLeftModule,
                                 SwerveModule backRighModule, Gyro gyro,
                                 SwerveDriveKinematics kinematics,
                                 Configuration config) {
        modules[0] = frontLeftModule;
        modules[1] = frontRightModule;
        modules[2] = backLeftModule;
        modules[3] = backRighModule;
        this.gyro = gyro;
        this.kinematics = kinematics;
        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry
         * readings. The numbers used below are robot specific, and should be
         * tuned.
         */
        poseEstimator = new SwerveDrivePoseEstimator(kinematics,
                                                     getGyroHeading(),
                                                     getModulePositions(),
                                                     new Pose2d(),
                                                     VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // TODO tune standard devs
                                                     VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // TODO tune standard devs
        this.config = config;
        holo = new HolonomicDriveController(new PIDController(config.kAutonDrivekP,
                                                              0, 0),
                                            new PIDController(config.kAutonDrivekP,
                                                              0, 0),
                                            new ProfiledPIDController(config.kAutonSteerKp,
                                                                      0, 0,
                                                                      new TrapezoidProfile.Constraints(config.kMaxRobotAngularVelocity,
                                                                                                       config.kMaxRobotAnglularAcceleration)));
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        poseEstimator.update(getGyroHeading(), getModulePositions());
    }

    public void updateOdometryWithVision(Pose2d botPose, double latency) {
        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp() - latency); // TODO determine latency of camera based on timestamps or other
    }

    public void setModuleStates(SwerveInput input, boolean creep) {
        var driveProp = creep ? config.kSlowDriveProp : config.kFastDriveProp;
        var steerProp = creep ? config.kSlowSteerProp : config.kFastSteerProp;
        var modMaxSpeed = driveProp * config.kMaxRobotSpeed;
        var modMaxAngularSpeed = steerProp * config.kMaxRobotAngularVelocity;
        input = handleStationary(input);
        switch (orientationChooser.getSelected()) {
            case "Field Oriented":
                setModuleStates(kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(input.m_translationX * modMaxSpeed, input.m_translationY * modMaxSpeed, input.m_rotation * modMaxAngularSpeed, getGyroHeading())));
                break;

            case "Robot Oriented":
                setModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(input.m_translationX * modMaxSpeed,
                                                                                  input.m_translationY * modMaxSpeed,
                                                                                  input.m_rotation * modMaxAngularSpeed)));
                break;
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, config.kMaxRobotSpeed);
        for (int i = 0; i < NUM_MODULES; i++) {
            modules[i].setDesiredState(swerveModuleStates[i]);
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
        zeroGyroscope(in.getRotation());
        poseEstimator.resetPosition(getGyroHeading(), getModulePositions(), in);
    }

    public void setKnownState(PathPlannerState initialState) {
        Pose2d startingPose
                        = new Pose2d(initialState.poseMeters.getTranslation(),
                                     initialState.holonomicRotation);
        setKnownPose(startingPose);
    }

    public Rotation2d getGyroHeading() {
        return gyro.getRotation2d().plus(gyroOffset);
    }

    public void zeroGyroscope(Rotation2d angle) {
        gyroOffset = angle.minus(getGyroHeading());
    }

    public void goToPose(PathPlannerState state) {
        setModuleStates(holo.calculate(getPose(), state.poseMeters, state.velocityMetersPerSecond, state.holonomicRotation));
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

    public SwerveModule[] getSwerveModules() {
        return modules;
    }

    public static class Configuration {
        final double kMaxRobotSpeed, kMaxRobotAngularVelocity,
                        kMaxRobotAnglularAcceleration;
        final double kAutonDrivekP, kAutonSteerKp;
        final double kFastDriveProp, kFastSteerProp, kSlowDriveProp,
                        kSlowSteerProp;

        public Configuration(double kMaxRobotSpeed,
                             double kMaxRobotAngularVelocity,
                             double kMaxRobotAnglularAcceleration,
                             double kAutonDrivekP, double kAutonSteerKp,
                             double kFastDriveProp, double kFastSteerProp,
                             double kSlowDriveProp, double kSlowSteerProp) {
            this.kMaxRobotSpeed = kMaxRobotSpeed;
            this.kMaxRobotAngularVelocity = kMaxRobotAngularVelocity;
            this.kMaxRobotAnglularAcceleration = kMaxRobotAnglularAcceleration;
            this.kAutonDrivekP = kAutonDrivekP;
            this.kAutonSteerKp = kAutonSteerKp;
            this.kFastDriveProp = kFastDriveProp;
            this.kFastSteerProp = kFastSteerProp;
            this.kSlowDriveProp = kSlowDriveProp;
            this.kSlowSteerProp = kSlowSteerProp;
        }
    }
}
