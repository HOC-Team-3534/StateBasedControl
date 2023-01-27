// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.statebasedcontroller.subsystem.general.swervedrive.swervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveModule {
  private final PIDController m_drivePIDController;
  private final ProfiledPIDController m_turningPIDController;
  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turnFeedforward;
  private final IDriveController m_driveController;
  private final ISteerController m_steerController;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotor     PWM drive motor
   * @param turningMotor   PWM turning motor
   * @param driveEncoder   DIO drive encoder
   * @param turningEncoder DIO turning encoder
   */
  public SwerveModule(ModuleConfiguration moduleConfiguration, Tunings tunings,
                      MotorController driveMotor, MotorController turningMotor,
                      Encoder driveEncoder, Encoder turningEncoder) {
    this(new BasicDriveController(driveMotor, driveEncoder),
         new BasicSteerController(turningMotor, turningEncoder),
         moduleConfiguration, tunings);
  }

  public SwerveModule(ModuleConfiguration moduleConfiguration, Tunings tunings,
                      WPI_TalonFX driveMotor, WPI_TalonFX steerMotor,
                      CANCoder absoluteEncoder, Rotation2d angleOffset) {
    this(new FalconDriveController(driveMotor,
                                   moduleConfiguration.isDriveInverted()),
         new FalconCANCoderSteerController(steerMotor, absoluteEncoder,
                                           angleOffset,
                                           moduleConfiguration.isSteerInverted()),
         moduleConfiguration, tunings);
  }

  private SwerveModule(IDriveController driveController,
                       ISteerController steerController,
                       ModuleConfiguration moduleConfiguration,
                       Tunings tunings) {
    m_driveController = driveController;
    m_steerController = steerController;
    m_drivePIDController = new PIDController(tunings.kDriveKp, 0, 0);
    m_turningPIDController
            = new ProfiledPIDController(tunings.kSteerKp, 0, 0,
                                        new TrapezoidProfile.Constraints(tunings.kMaxAngularVelocity,
                                                                         tunings.kMaxAngularAcceleration));
    m_driveFeedforward
            = new SimpleMotorFeedforward(tunings.kDriveKs, tunings.kDriveKv);
    m_turnFeedforward
            = new SimpleMotorFeedforward(tunings.kSteerKs, tunings.kSteerKv);
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveController.setDistancePerEncoderShaftRotation(Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction()); // TODO
                                                                                                                                                    // move
                                                                                                                                                    // to
                                                                                                                                                    // the
                                                                                                                                                    // following
                                                                                                                                                    // three
                                                                                                                                                    // to
                                                                                                                                                    // the
                                                                                                                                                    // interface
                                                                                                                                                    // Set the distance (in this case, angle) in radians per pulse for the turning
                                                                                                                                                    // encoder.
                                                                                                                                                    // This is the the angle through an entire rotation (2 * pi) divided by the
                                                                                                                                                    // encoder resolution.
    steerController.setDistancePerEncoderShaftRotation(2 * Math.PI * moduleConfiguration.getSteerReduction());
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveController.getRate(),
                                 m_steerController.getRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveController.getDistance(),
                                    m_steerController.getRotation2d());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state
            = SwerveModuleState.optimize(desiredState, m_steerController.getRotation2d());
    // Calculate the drive output from the drive PID controller.
    final double driveOutput
            = m_drivePIDController.calculate(m_driveController.getRate(), state.speedMetersPerSecond);
    final double driveFeedforward
            = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput
            = m_turningPIDController.calculate(m_steerController.getRotation2d().getRadians(), state.angle.getRadians());
    final double turnFeedforward
            = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    m_driveController.setVoltage(driveOutput + driveFeedforward);
    m_steerController.setVoltage(turnOutput + turnFeedforward);
  }

  private static class BasicDriveController implements IDriveController {
    final MotorController driveMotor;
    final Encoder driveEncoder;
    final int kEncoderResolution = 4096;

    BasicDriveController(MotorController driveMotor, Encoder driveEncoder) {
      this.driveMotor = driveMotor;
      this.driveEncoder = driveEncoder;
    }

    @Override
    public void setDistancePerEncoderShaftRotation(double motorRotsToMeters) {
      driveEncoder.setDistancePerPulse(motorRotsToMeters / kEncoderResolution);
    }

    @Override
    public double getRate() {
      return driveEncoder.getRate();
    }

    @Override
    public double getDistance() {
      return driveEncoder.getDistance();
    }

    @Override
    public void setVoltage(double voltage) {
      driveMotor.setVoltage(voltage);
    }
  }

  private static class BasicSteerController implements ISteerController {
    final MotorController steerMotor;
    final Encoder steerEncoder;
    final int kEncoderResolution = 4096;

    BasicSteerController(MotorController steerMotor, Encoder steerEncoder) {
      this.steerMotor = steerMotor;
      this.steerEncoder = steerEncoder;
    }

    @Override
    public void setDistancePerEncoderShaftRotation(double motorRotsToRadians) {
      steerEncoder.setDistancePerPulse(motorRotsToRadians / kEncoderResolution);
    }

    @Override
    public Rotation2d getRotation2d() {
      return new Rotation2d(steerEncoder.getDistance());
    }

    @Override
    public void setVoltage(double voltage) {
      steerMotor.setVoltage(voltage);
    }
  }

  private static class FalconDriveController implements IDriveController {
    final WPI_TalonFX driveMotor;
    final int kEncoderResolution = 2048;
    double kMetersPerTick;

    FalconDriveController(WPI_TalonFX driveMotor, boolean inverted) {
      this.driveMotor = driveMotor;
      this.driveMotor.setInverted(inverted);
    }

    @Override
    public void setDistancePerEncoderShaftRotation(double motorRotsToMeters) {
      this.kMetersPerTick = motorRotsToMeters / kEncoderResolution;
    }

    @Override
    public double getRate() {
      return driveMotor.getSelectedSensorVelocity() * 10 * kMetersPerTick;
    }

    @Override
    public double getDistance() {
      return driveMotor.getSelectedSensorPosition() * kMetersPerTick;
    }

    @Override
    public void setVoltage(double voltage) {
      driveMotor.setVoltage(voltage);
    }
  }

  private static class FalconCANCoderSteerController implements ISteerController {
    final WPI_TalonFX steerMotor;
    final CANCoder absoluteEncoder;
    final Rotation2d angleOffset;
    final int kEncoderResolution = 2048;
    double kRadiansPerTick;
    double resetIteration;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY
            = Math.toRadians(0.5);

    FalconCANCoderSteerController(WPI_TalonFX steerMotor,
                                  CANCoder absoluteEncoder,
                                  Rotation2d angleOffset, boolean inverted) {
      this.steerMotor = steerMotor;
      this.steerMotor.setInverted(inverted);
      this.absoluteEncoder = absoluteEncoder;
      this.angleOffset = angleOffset;
    }

    @Override
    public void setDistancePerEncoderShaftRotation(double motorRotsToRadians) {
      this.kRadiansPerTick = motorRotsToRadians / kEncoderResolution;
    }

    @Override
    public Rotation2d getRotation2d() {
      return new Rotation2d(-steerMotor.getSelectedSensorPosition() * kRadiansPerTick);
    }

    @Override
    public void setVoltage(double voltage) {
      if (Math.abs(steerMotor.getSelectedSensorVelocity() * 10 * kRadiansPerTick) < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
        if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
          resetIteration = 0;
          double absoluteAngle
                  = Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()).minus(angleOffset).getRadians();
          // Be aware. DO NOT USE kI or kD with the PID controller for steering or this
          // reseed will cause issues
          steerMotor.setSelectedSensorPosition(-absoluteAngle / kRadiansPerTick);
        }
      } else {
        resetIteration = 0;
      }
      steerMotor.setVoltage(voltage);
    }
  }

  public static class Tunings {
    final double kMaxSpeed, kMaxAngularVelocity, kMaxAngularAcceleration;
    final double kDriveKp, kDriveKs, kDriveKv, kSteerKp, kSteerKs, kSteerKv;

    public Tunings(double maxSpeed, double maxAngVel, double maxAngAcc,
                   double drvKp, double drvKs, double drvKv, double strKp,
                   double strKs, double strKv) {
      this.kMaxSpeed = maxSpeed;
      this.kMaxAngularVelocity = maxAngVel;
      this.kMaxAngularAcceleration = maxAngAcc;
      this.kDriveKp = drvKp;
      this.kDriveKs = drvKs;
      this.kDriveKv = drvKv;
      this.kSteerKp = strKp;
      this.kSteerKs = strKs;
      this.kSteerKv = strKv;
    }
  }
}

interface IDriveController {
  /**
   * @param motorRotsToMeters the conversion factor between drive encoder
   *                          rotations and meters crossed by the wheel
   */
  void setDistancePerEncoderShaftRotation(double motorRotsToMeters);

  /**
   * @return the velocity in meters per second
   */
  double getRate();

  /**
   * @return the distance in meters
   */
  double getDistance();

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
  void setDistancePerEncoderShaftRotation(double motorRotsToRadians);

  Rotation2d getRotation2d();

  void setVoltage(double voltage);
}
