package frc.statebasedcontroller.subsystem.general.swervedrive.swervelib;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    public static double maxSpeed;
    public static double moduleMaxAngularVel, moduleMaxAngularAccel;
    public static double robotMaxAngularVel, robotMaxAngularAccel;
    public static SwerveDriveKinematics kinematics;
    public static SDSModuleConfiguration moduleConfiguration;
    public static double driveKP, driveKS, driveKV, driveKA;
    //only need to set steer values if using basic swerve module
    public static double steerKP, steerKS, steerKV, steerKA, steerKI, steerKD,
                    steerKF;
    public static double autonDriveKP, autonSteerKP;
    public static double fastDriveProp, fastSteerProp, slowDriveProp,
                    slowSteerProp;
    public static double modulePoseEstXStdDev = 0.01,
                    modulePoseEstYStdDev = 0.01, visionPoseEstXStdDev = 0.01,
                    visionPoseEstYStdDev = 0.01;
    public static Rotation2d modulePoseEstAngleStdDev
                    = Rotation2d.fromDegrees(0.5);
    public static Rotation2d visionPoseEstAngleStdDev
                    = Rotation2d.fromDegrees(5);
    public static NeutralMode driveNeutralMode = NeutralMode.Brake,
                    angleNeutralMode = NeutralMode.Coast;
    public static IdleMode driveIdleMode
                    = (driveNeutralMode == NeutralMode.Brake) ? IdleMode.kBrake
                                                              : IdleMode.kCoast,
                    angleIdleMode = (angleNeutralMode == NeutralMode.Brake) ? IdleMode.kBrake
                                                                            : IdleMode.kCoast;
    public static boolean angleEnableCurrentLimit = true,
                    driveEnableCurrentLimit = true;
    public static int angleContinuousCurrentLimit = 25,
                    anglePeakCurrentLimit = 40,
                    driveContinuousCurrentLimit = 35,
                    drivePeakCurrentLimit = 60;
    public static double anglePeakCurrentDuration = 0.1,
                    drivePeakCurrentDuration = 0.1;
    public static double openLoopRamp = 0.25, closedLoopRamp = 0.0;
    public static AbsoluteSensorRange absoluteSensorRange
                    = AbsoluteSensorRange.Unsigned_0_to_360;
    public static SensorInitializationStrategy absoluteInitStrategy
                    = SensorInitializationStrategy.BootToAbsolutePosition;
    public static SensorTimeBase absoluteSensorTimeBase
                    = SensorTimeBase.PerSecond;
    public static TalonFXConfiguration swerveDriveFXConfig, swerveAngleFXconfig;
    public static CANCoderConfiguration swerveCanCoderConfig;

    public static void
                    fillNecessaryConstantsForFalcon(double maxSpeed_,
                                                    double robotMaxAngularVel_,
                                                    double robotMaxAngularAccel_,
                                                    SwerveDriveKinematics kinematics_,
                                                    SDSModuleConfiguration moduleConfiguration_,
                                                    double driveKP_,
                                                    double driveKS_,
                                                    double driveKV_,
                                                    double driveKA_,
                                                    double autonDriveKP_,
                                                    double autonSteerKP_,
                                                    double fastDriveProp_,
                                                    double fastSteerProp_,
                                                    double slowDriveProp_,
                                                    double slowSteerProp_) {
        maxSpeed = maxSpeed_;
        robotMaxAngularVel = robotMaxAngularVel_;
        robotMaxAngularAccel = robotMaxAngularAccel_;
        kinematics = kinematics_;
        moduleConfiguration = moduleConfiguration_;
        driveKP = driveKP_;
        driveKS = driveKS_;
        driveKV = driveKV_;
        driveKA = driveKA_;
        autonDriveKP = autonDriveKP_;
        autonSteerKP = autonSteerKP_;
        fastDriveProp = fastDriveProp_;
        fastSteerProp = fastSteerProp_;
        slowDriveProp = slowDriveProp_;
        slowSteerProp = slowSteerProp_;
    }

    /**
     * Must be called AFTER values have been set
     */
    public static void createSwerveConstants() {
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig.supplyCurrLimit
                        = new SupplyCurrentLimitConfiguration(driveEnableCurrentLimit,
                                                              driveContinuousCurrentLimit,
                                                              drivePeakCurrentLimit,
                                                              drivePeakCurrentDuration);
        swerveDriveFXConfig.slot0.kP = driveKP;
        swerveDriveFXConfig.openloopRamp = openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = closedLoopRamp;
        swerveAngleFXconfig = new TalonFXConfiguration();
        swerveAngleFXconfig.supplyCurrLimit
                        = new SupplyCurrentLimitConfiguration(angleEnableCurrentLimit,
                                                              angleContinuousCurrentLimit,
                                                              anglePeakCurrentLimit,
                                                              anglePeakCurrentDuration);
        swerveAngleFXconfig.slot0.kP = moduleConfiguration.angleKP;
        swerveAngleFXconfig.slot0.kI = moduleConfiguration.angleKI;
        swerveAngleFXconfig.slot0.kD = moduleConfiguration.angleKD;
        swerveAngleFXconfig.slot0.kF = moduleConfiguration.angleKF;
        swerveCanCoderConfig = new CANCoderConfiguration();
        swerveCanCoderConfig.absoluteSensorRange = absoluteSensorRange;
        swerveCanCoderConfig.sensorDirection
                        = moduleConfiguration.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = absoluteInitStrategy;
        swerveCanCoderConfig.sensorTimeBase = absoluteSensorTimeBase;
    }
}
