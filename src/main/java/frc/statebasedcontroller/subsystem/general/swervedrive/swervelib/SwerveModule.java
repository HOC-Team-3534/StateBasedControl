// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.statebasedcontroller.subsystem.general.swervedrive.swervelib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule implements ISwerveModule {
  final IDriveController m_driveController;
  final ISteerController m_steerController;
  final ModuleType moduleType;
  /* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
  private double simSpeedCache, simPositionCache;
  private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

  enum ModuleType {
    FalconFalconCanCoder,
    FalconNEOCanCoder,
    NEOFalconCanCoder,
    NEONEOCanCoder,
    Basic
  }

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotor     PWM drive motor
   * @param turningMotor   PWM turning motor
   * @param driveEncoder   DIO drive encoder
   * @param turningEncoder DIO turning encoder
   */
  public SwerveModule(MotorController driveMotor, MotorController turningMotor,
                      Encoder driveEncoder, Encoder turningEncoder) {
    m_driveController = new BasicDriveController(driveMotor, driveEncoder);
    m_steerController = new BasicSteerController(turningMotor, turningEncoder);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.Basic;
  }

  public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX steerMotor,
                      CANCoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new FalconDriveController(driveMotor);
    m_steerController
            = new FalconCANCoderSteerController(steerMotor, absoluteEncoder,
                                                angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.FalconFalconCanCoder;
  }

  public SwerveModule(WPI_TalonFX driveMotor, CANSparkMax steerMotor,
                      CANCoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new FalconDriveController(driveMotor);
    m_steerController
            = new NEOCANCoderSteerController(steerMotor, absoluteEncoder,
                                             angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.FalconNEOCanCoder;
  }

  public SwerveModule(CANSparkMax driveMotor, WPI_TalonFX steerMotor,
                      CANCoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new NEODriveController(driveMotor);
    m_steerController
            = new FalconCANCoderSteerController(steerMotor, absoluteEncoder,
                                                angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.NEOFalconCanCoder;
  }

  public SwerveModule(CANSparkMax driveMotor, CANSparkMax steerMotor,
                      CANCoder absoluteEncoder, Rotation2d angleOffset) {
    m_driveController = new NEODriveController(driveMotor);
    m_steerController
            = new NEOCANCoderSteerController(steerMotor, absoluteEncoder,
                                             angleOffset);
    m_driveController.config();
    m_steerController.config();
    moduleType = ModuleType.FalconNEOCanCoder;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState((RobotBase.isReal()) ? m_driveController.getVelocity()
                                                      : simSpeedCache,
                                 (RobotBase.isReal()) ? m_steerController.getAngle()
                                                      : simAngleCache);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    simPositionCache += simSpeedCache * 0.020;
    return new SwerveModulePosition((RobotBase.isReal()) ? m_driveController.getDistance()
                                                         : simPositionCache,
                                    (RobotBase.isReal()) ? m_steerController.getAngle()
                                                         : simAngleCache);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   * @param isOpenLoop   use percent output instead of velocity control.
   */
  public void setDesiredState(SwerveModuleState desiredState,
                              boolean isOpenLoop) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    switch (moduleType) {
      case Basic:
        desiredState
                = SwerveModuleState.optimize(desiredState, getState().angle);
        break;

      default:
        //should work for REV too
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        break;
    }
    m_driveController.setSpeed(desiredState, isOpenLoop);
    m_steerController.setAngle(desiredState);
    simSpeedCache = desiredState.speedMetersPerSecond;
    simAngleCache = desiredState.angle;
  }

  /**
   * Use for characterizing the drive only. Steer set to 0
   * 
   * @param voltage the voltage to set the drive motor to
   */
  public void setDriveVoltageForCharacterization(double voltage) {
    m_driveController.setVoltage(voltage);
    switch (moduleType) {
      case FalconFalconCanCoder:
        // need to send more than 1% speed otherwise it wont set the angle b/c of the nice 
        // filter to  reduce jittering, but it doesnt actually set the speed for drive
        // to 1 meter per second
        m_steerController.setAngle(new SwerveModuleState(1, new Rotation2d()));
        break;

      default:
        m_steerController.setVoltage(0);
        break;
    }
  }

  /**
   * Use for characterizing a steering motor only. Drive set to 0
   * 
   * @param voltage the voltage to set the steer motor to
   */
  public void setSteerVoltageForCharacterization(double voltage) {
    m_steerController.setVoltage(voltage);
    m_driveController.setVoltage(0);
  }

  /**
   * @return the drive motor voltage and velocity in meters per second of the
   *         wheel
   */
  public Pair<Double, Double> getDriveVoltageAndRate() {
    return new Pair<>(m_driveController.getVoltage(),
                      m_driveController.getVelocity());
  }

  /**
   * @return the steer motor voltage and angular velocity in radians per second
   *         of the wheel vertical axel
   */
  public Pair<Double, Double> getSteerVoltageAndRate() {
    return new Pair<>(m_steerController.getVoltage(),
                      m_steerController.getRate().getRadians());
  }

  private static class BasicDriveController implements IDriveController {
    final MotorController driveMotor;
    final Encoder driveEncoder;
    final int kEncoderResolution = 4096;
    double lastVoltage;
    final PIDController m_drivePIDController
            = new PIDController(SwerveConstants.driveKP, 0, 0);
    final SimpleMotorFeedforward m_driveFeedforward
            = new SimpleMotorFeedforward(SwerveConstants.driveKS,
                                         SwerveConstants.driveKV,
                                         SwerveConstants.driveKA);

    BasicDriveController(MotorController driveMotor, Encoder driveEncoder) {
      this.driveMotor = driveMotor;
      this.driveEncoder = driveEncoder;
    }

    @Override
    public void config() {
      var config = SwerveConstants.moduleConfiguration;
      driveEncoder.setDistancePerPulse(Math.PI * config.wheelDiameter / config.driveGearRatio / kEncoderResolution);
    }

    @Override
    public double getVelocity() {
      return driveEncoder.getRate();
    }

    @Override
    public double getDistance() {
      return driveEncoder.getDistance();
    }

    @Override
    public void setVoltage(double voltage) {
      driveMotor.setVoltage(voltage);
      lastVoltage = voltage;
    }

    @Override
    public double getVoltage() {
      return lastVoltage;
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
      if (isOpenLoop) {
        double percentOutput
                = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
        driveMotor.setVoltage(percentOutput);
      } else {
        // Calculate the drive output from the drive PID controller.
        double driveOutput
                = m_drivePIDController.calculate(getVelocity(), desiredState.speedMetersPerSecond);
        double driveFeedforward
                = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        setVoltage(driveOutput + driveFeedforward);
      }
    }
  }

  private static class BasicSteerController implements ISteerController {
    final MotorController steerMotor;
    final Encoder steerEncoder;
    final int kEncoderResolution = 4096;
    double lastVoltage;
    private final SimpleMotorFeedforward m_turnFeedforward
            = new SimpleMotorFeedforward(SwerveConstants.steerKS,
                                         SwerveConstants.steerKV);
    private final ProfiledPIDController m_turningPIDController
            = new ProfiledPIDController(SwerveConstants.steerKP, 0, 0,
                                        new TrapezoidProfile.Constraints(SwerveConstants.moduleMaxAngularVel,
                                                                         SwerveConstants.moduleMaxAngularAccel));

    BasicSteerController(MotorController steerMotor, Encoder steerEncoder) {
      this.steerMotor = steerMotor;
      this.steerEncoder = steerEncoder;
    }

    @Override
    public void config() {
      var config = SwerveConstants.moduleConfiguration;
      steerEncoder.setDistancePerPulse(2 * Math.PI / config.angleGearRatio / kEncoderResolution);
      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public Rotation2d getAngle() {
      return new Rotation2d(steerEncoder.getDistance());
    }

    @Override
    public void setVoltage(double voltage) {
      steerMotor.setVoltage(voltage);
      lastVoltage = voltage;
    }

    @Override
    public Rotation2d getRate() {
      return new Rotation2d(steerEncoder.getRate());
    }

    @Override
    public double getVoltage() {
      return lastVoltage;
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput
              = m_turningPIDController.calculate(getAngle().getRadians(), desiredState.angle.getRadians());
      final double turnFeedforward
              = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
      setVoltage(turnOutput + turnFeedforward);
    }
  }

  private static class FalconDriveController implements IDriveController {
    final WPI_TalonFX driveMotor;
    final SimpleMotorFeedforward m_driveFeedforward
            = new SimpleMotorFeedforward(SwerveConstants.driveKS,
                                         SwerveConstants.driveKV,
                                         SwerveConstants.driveKA);

    FalconDriveController(WPI_TalonFX driveMotor) {
      this.driveMotor = driveMotor;
    }

    @Override
    public void config() {
      driveMotor.configFactoryDefault();
      driveMotor.configAllSettings(SwerveConstants.swerveDriveFXConfig);
      driveMotor.setInverted(SwerveConstants.moduleConfiguration.driveMotorInvert);
      driveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
      driveMotor.setSelectedSensorPosition(0);
    }

    @Override
    public double getVelocity() {
      var config = SwerveConstants.moduleConfiguration;
      return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), config.wheelCircumference, config.driveGearRatio);
    }

    @Override
    public double getDistance() {
      var config = SwerveConstants.moduleConfiguration;
      return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), config.wheelCircumference, config.driveGearRatio);
    }

    @Override
    public void setVoltage(double voltage) {
      driveMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.driveMotorInvert) ? -1
                                                                                              : 1));
    }

    @Override
    public double getVoltage() {
      return driveMotor.getMotorOutputVoltage() * ((SwerveConstants.moduleConfiguration.driveMotorInvert) ? -1
                                                                                                          : 1);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
      if (isOpenLoop) {
        double percentOutput
                = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
      } else {
        var config = SwerveConstants.moduleConfiguration;
        double velocity
                = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, config.wheelCircumference, config.driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
      }
    }
  }

  private static class FalconCANCoderSteerController implements ISteerController {
    final WPI_TalonFX steerMotor;
    final CANCoder absoluteEncoder;
    final Rotation2d angleOffset;
    double resetIteration;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY
            = Math.toRadians(0.5);
    Rotation2d lastAngle;

    FalconCANCoderSteerController(WPI_TalonFX steerMotor,
                                  CANCoder absoluteEncoder,
                                  Rotation2d angleOffset) {
      this.steerMotor = steerMotor;
      this.absoluteEncoder = absoluteEncoder;
      this.angleOffset = angleOffset;
    }

    @Override
    public void config() {
      absoluteEncoder.configFactoryDefault();
      absoluteEncoder.configAllSettings(SwerveConstants.swerveCanCoderConfig);
      steerMotor.configFactoryDefault();
      steerMotor.configAllSettings(SwerveConstants.swerveAngleFXconfig);
      steerMotor.setInverted(SwerveConstants.moduleConfiguration.angleMotorInvert);
      steerMotor.setNeutralMode(SwerveConstants.angleNeutralMode);
      resetToAbsolute();
      lastAngle = getAngle();
    }

    Rotation2d getCanCoder() {
      return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), SwerveConstants.moduleConfiguration.angleGearRatio));
    }

    @Override
    public void setVoltage(double voltage) {
      if (Math.abs(getRate().getRadians()) < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
        if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
          resetIteration = 0;
          resetToAbsolute();
        }
      } else {
        resetIteration = 0;
      }
      steerMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.angleMotorInvert) ? -1
                                                                                              : 1));
    }

    void resetToAbsolute() {
      double absolutePosition
              = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.moduleConfiguration.angleGearRatio);
      steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    @Override
    public Rotation2d getRate() {
      return Rotation2d.fromDegrees(Conversions.falconToDegreesPerSecond(steerMotor.getSelectedSensorVelocity(), SwerveConstants.moduleConfiguration.angleGearRatio));
    }

    @Override
    public double getVoltage() {
      return steerMotor.getMotorOutputVoltage() * ((SwerveConstants.moduleConfiguration.angleMotorInvert) ? -1
                                                                                                          : 1);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
      Rotation2d angle
              = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01)) ? lastAngle
                                                                                                   : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
      steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.moduleConfiguration.angleGearRatio));
      lastAngle = angle;
    }
  }

  private static class NEODriveController implements IDriveController {
    final CANSparkMax driveMotor;
    final RelativeEncoder driveEncoder;
    final SparkMaxPIDController drivePID;
    final SimpleMotorFeedforward m_driveFeedforward
            = new SimpleMotorFeedforward(SwerveConstants.driveKS,
                                         SwerveConstants.driveKV,
                                         SwerveConstants.driveKA);

    NEODriveController(CANSparkMax driveMotor) {
      this.driveMotor = driveMotor;
      this.driveEncoder = driveMotor.getEncoder();
      this.drivePID = driveMotor.getPIDController();
    }

    @Override
    public void config() {
      driveMotor.restoreFactoryDefaults();
      driveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
      driveMotor.setSecondaryCurrentLimit(SwerveConstants.drivePeakCurrentLimit);
      driveMotor.setInverted(SwerveConstants.moduleConfiguration.driveMotorInvert);
      driveMotor.setIdleMode(SwerveConstants.driveIdleMode);
      driveMotor.setOpenLoopRampRate(SwerveConstants.openLoopRamp);
      driveMotor.setClosedLoopRampRate(SwerveConstants.closedLoopRamp);
      driveEncoder.setPositionConversionFactor(1 / SwerveConstants.moduleConfiguration.driveGearRatio * SwerveConstants.moduleConfiguration.wheelCircumference);
      driveEncoder.setVelocityConversionFactor(1 / SwerveConstants.moduleConfiguration.driveGearRatio * SwerveConstants.moduleConfiguration.wheelCircumference / 60.0);
      driveEncoder.setPosition(0);
      drivePID.setP(SwerveConstants.driveKP);
    }

    @Override
    public double getVoltage() {
      return driveMotor.getAppliedOutput();
    }

    @Override
    public double getVelocity() {
      return driveEncoder.getVelocity();
    }

    @Override
    public double getDistance() {
      return driveEncoder.getPosition();
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
      if (isOpenLoop) {
        double percentOutput
                = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
        driveMotor.set(percentOutput);
      } else {
        drivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
      }
    }

    @Override
    public void setVoltage(double voltage) {
      driveMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.driveMotorInvert) ? -1
                                                                                              : 1));
    }
  }

  private static class NEOCANCoderSteerController implements ISteerController {
    final CANSparkMax steerMotor;
    final RelativeEncoder steerEncoder;
    final SparkMaxPIDController steerPID;
    final CANCoder absoluteEncoder;
    final Rotation2d angleOffset;
    Rotation2d lastAngle;

    NEOCANCoderSteerController(CANSparkMax steerMotor, CANCoder absoluteEncoder,
                               Rotation2d angleOffset) {
      this.steerMotor = steerMotor;
      this.steerEncoder = steerMotor.getEncoder();
      this.steerPID = steerMotor.getPIDController();
      this.absoluteEncoder = absoluteEncoder;
      this.angleOffset = angleOffset;
    }

    @Override
    public void config() {
      absoluteEncoder.configFactoryDefault();
      absoluteEncoder.configAllSettings(SwerveConstants.swerveCanCoderConfig);
      steerMotor.restoreFactoryDefaults();
      steerMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
      steerMotor.setSecondaryCurrentLimit(SwerveConstants.anglePeakCurrentLimit);
      steerMotor.setInverted(SwerveConstants.moduleConfiguration.angleMotorInvert);
      steerMotor.setIdleMode(SwerveConstants.angleIdleMode);
      steerEncoder.setPositionConversionFactor(1 / SwerveConstants.moduleConfiguration.angleGearRatio * 360.0);
      steerEncoder.setVelocityConversionFactor(1 / SwerveConstants.moduleConfiguration.angleGearRatio * 360.0 / 60.0);
      resetToAbsolute();
      steerPID.setP(SwerveConstants.steerKP);
      steerPID.setI(SwerveConstants.steerKI);
      steerPID.setD(SwerveConstants.steerKD);
      steerPID.setFF(SwerveConstants.steerKF);
      lastAngle = getAngle();
    }

    void resetToAbsolute() {
      steerEncoder.setPosition(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }

    Rotation2d getCanCoder() {
      return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
      Rotation2d angle
              = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01)) ? lastAngle
                                                                                                   : desiredState.angle; //Prevent rotating module if speed is less than 1%. Prevents Jittering.
      steerPID.setReference(angle.getDegrees(), ControlType.kPosition);
      lastAngle = angle;
    }

    @Override
    public double getVoltage() {
      return steerMotor.getAppliedOutput();
    }

    @Override
    public Rotation2d getRate() {
      return Rotation2d.fromDegrees(steerEncoder.getVelocity());
    }

    @Override
    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }

    @Override
    public void setVoltage(double voltage) {
      steerMotor.setVoltage(voltage * ((SwerveConstants.moduleConfiguration.angleMotorInvert) ? -1
                                                                                              : 1));
    }
  }
}

interface IDriveController {
  /**
   * @param motorRotsToMeters the conversion factor between drive encoder
   *                          rotations and meters crossed by the wheel
   */
  void config();

  double getVoltage();

  /**
   * @return the velocity in meters per second
   */
  double getVelocity();

  /**
   * @return the distance in meters
   */
  double getDistance();

  void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop);

  /**
   * @param voltage the voltage to send to the drive motor
   */
  void setVoltage(double voltage);
}

interface ISteerController {
  /**
   * @param motorRotsToRadians the conversion factor from rotations of the steer
   *                           encoder to radians turned at the module axle
   */
  void config();

  void setAngle(SwerveModuleState desiredState);

  double getVoltage();

  Rotation2d getRate();

  Rotation2d getAngle();

  void setVoltage(double voltage);
}
