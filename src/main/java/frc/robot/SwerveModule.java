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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkPIDController m_drivePIDController;
  private final SparkPIDController m_turningPIDController;

  private final CANcoder m_turningCANCoder;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN ID of the drive motor controller.
   * @param turningMotorChannel CAN ID of the turning motor controller.
   * @param turningAbsoluteEncoderChannel CAN ID of absolute encoder
   * @param turningAbsoluteEncoderOffset Offset angle of the absolute encoder
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningAbsoluteEncoderChannel,
      double turningAbsoluteEncoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_driveMotor.enableVoltageCompensation(12.0);

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

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMaxInput(Math.PI);
    m_turningPIDController.setPositionPIDWrappingMinInput(-Math.PI);

    m_turningCANCoder = new CANcoder(turningAbsoluteEncoderChannel);
    CANcoderConfiguration m_configs = new CANcoderConfiguration();
    m_configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    m_configs.MagnetSensor.MagnetOffset = turningAbsoluteEncoderOffset;
    m_configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_turningCANCoder.getConfigurator().apply(m_configs);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);

    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurnPositionConversionFactor);

    double absPosition = m_turningCANCoder.getAbsolutePosition().getValue() * (2.0 * Math.PI);
    m_turningEncoder.setPosition(absPosition);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turningEncoder.getPosition());

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
}
