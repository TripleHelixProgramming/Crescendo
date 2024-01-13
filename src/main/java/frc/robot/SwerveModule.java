// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  public final String moduleName;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;

  private final SparkPIDController m_drivePIDController;
  private final SparkPIDController m_turningPIDController;

  private final CANcoder m_turningAbsEncoder;
  private final CANcoderConfiguration m_turningAbsEncoderConfig;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param moduleName Name of the module
   * @param driveMotorChannel CAN ID of the drive motor controller
   * @param turningMotorChannel CAN ID of the turning motor controller
   * @param turningAbsoluteEncoderChannel CAN ID of absolute encoder
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAbsoluteEncoderChannel) {
    moduleName = name;

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_driveMotor.enableVoltageCompensation(12.0);

    m_turningMotor.setInverted(true);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_turningPIDController = m_turningMotor.getPIDController();

    m_drivePIDController.setP(ModuleConstants.kDriveP);
    m_drivePIDController.setI(ModuleConstants.kDriveI);
    m_drivePIDController.setD(ModuleConstants.kDriveD);
    // m_drivePIDController.setIZone();
    m_drivePIDController.setFF(ModuleConstants.kDriveFF);
    // m_drivePIDController.setOutputRange();

    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    // m_turningPIDController.setIZone();
    // m_turningPIDController.setFF();
    // m_turningPIDController.setOutputRange();

    m_turningPIDController.setSmartMotionMaxVelocity(
        ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond, 0);
    // m_turningPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
    m_turningPIDController.setSmartMotionMaxAccel(
        ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared, 0);
    // m_turningPIDController.setSmartMotionAllowedClosedLoopError(0.1, 0);

    // Limit the PID Controller's range to (-pi, pi], with continuous wrapping
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Math.PI);
    m_turningPIDController.setPositionPIDWrappingMinInput(-Math.PI);

    m_turningAbsEncoder = new CANcoder(turningAbsoluteEncoderChannel);
    m_turningAbsEncoderConfig = new CANcoderConfiguration();
    m_turningAbsEncoder.getConfigurator().refresh(m_turningAbsEncoderConfig);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningRelativeEncoder = m_turningMotor.getEncoder();

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);

    m_turningRelativeEncoder.setPositionConversionFactor(
        ModuleConstants.kTurnPositionConversionFactor);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getRelativeTurningPosition());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getRelativeDrivePosition(), getRelativeTurningPosition());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = getRelativeTurningPosition();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    m_drivePIDController.setReference(
        state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(
        state.angle.getRadians(), CANSparkMax.ControlType.kSmartMotion);
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  /**
   * @return The relative drive position of the module
   */
  public double getRelativeDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  /**
   * @return The relative turning angle of the module
   */
  public Rotation2d getRelativeTurningPosition() {
    double relativePositionRadians = m_turningRelativeEncoder.getPosition();
    return Rotation2d.fromRadians(MathUtil.angleModulus(relativePositionRadians));
    //return Rotation2d.fromRadians(m_turningRelativeEncoder.getPosition());
  }

  /**
   * @return The absolute turning angle of the module
   */
  public Rotation2d getAbsTurningPosition() {
    //double absPositonRotations = m_turningAbsEncoder.getPosition().getValue();
    //return Rotation2d.fromRotations(MathUtil.inputModulus(absPositonRotations, -0.5, 0.5));
    return Rotation2d.fromRotations(m_turningAbsEncoder.getPosition().getValue());
  }

  /**
   * Updates the relative turning encoder to match the absolute measurement of the module turnin
   * angle.
   */
  public void syncTurningEncoders() {
    m_turningRelativeEncoder.setPosition(getAbsTurningPosition().getRadians());
  }

  /**
   * Adjusts the offset of the absolute turning encoder to align the wheels with an alignment
   * device. To be performed upon a hardware change (e.g. when a swerve module or absolute turning
   * encoder has been swapped.)
   */
  public void setAbsTurningEncoderZero() {
    Rotation2d currentMagnetOffset = getMagnetOffset();
    Rotation2d newMagnetOffset = currentMagnetOffset.minus(getAbsTurningPosition());
    m_turningAbsEncoderConfig.MagnetSensor.MagnetOffset = newMagnetOffset.getRotations();

    m_turningAbsEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_turningAbsEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.Clockwise_Positive;
    m_turningAbsEncoder.getConfigurator().apply(m_turningAbsEncoderConfig);

    m_turningRelativeEncoder.setPosition(0.0);
  }

  /**
   * @return The magnet offset of the module's absolute turning encoder
   */
  public Rotation2d getMagnetOffset() {
    m_turningAbsEncoder.getConfigurator().refresh(m_turningAbsEncoderConfig);
    return Rotation2d.fromRotations(m_turningAbsEncoderConfig.MagnetSensor.MagnetOffset);
  }

  /**
   * @return The name of the swerve module
   */
  public String getName() {
    return moduleName;
  }
}
