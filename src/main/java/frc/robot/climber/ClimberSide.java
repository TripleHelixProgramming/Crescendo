package frc.robot.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.ClimberConstants;

public class ClimberSide {

  private final CANSparkMax m_climberMover;
  private final RelativeEncoder m_climberRelativeEncoder;
  private final SparkPIDController m_climberPIDController;

  private double positionSetpoint;
  private boolean hasFinishedCalibrating = false;

  public ClimberSide(int climberMotorChannel) {
    m_climberMover = new CANSparkMax(climberMotorChannel, MotorType.kBrushless);

    m_climberMover.restoreFactoryDefaults();

    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_climberMover.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.kUpperLimit);
    m_climberMover.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.kLowerLimit);

    m_climberMover.setIdleMode(IdleMode.kBrake);
    m_climberMover.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
    m_climberMover.setInverted(false);

    m_climberPIDController = m_climberMover.getPIDController();

    m_climberPIDController.setP(ClimberConstants.kClimberP);
    m_climberPIDController.setI(ClimberConstants.kClimberI);
    m_climberPIDController.setD(ClimberConstants.kClimberD);

    m_climberRelativeEncoder = m_climberMover.getEncoder();

    m_climberRelativeEncoder.setPositionConversionFactor(
        ClimberConstants.kClimberPositionConversionFactor);
    m_climberRelativeEncoder.setVelocityConversionFactor(
        ClimberConstants.kClimberVelocityConversionFactor);
  }

  // public void setVelocity(double targetVelocity) {
  //   m_climberPIDController.setReference(targetVelocity, ControlType.kVelocity);
  // }

  public void setVoltage(double targetVoltage) {
    m_climberMover.setVoltage(targetVoltage);
  }

  public void setPosition(double targetPosition) {
    this.positionSetpoint = targetPosition;
    m_climberPIDController.setReference(positionSetpoint, ControlType.kPosition);
  }

  public void configureUpperLimit(boolean upperLImitEnabled) {
    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLImitEnabled);
  }

  public void resetEncoder() {
    m_climberRelativeEncoder.setPosition(0.0);
  }

  public boolean getUpperLimitDetected() {
    return m_climberMover.getOutputCurrent() > ClimberConstants.kClimberMotorCurrentHardStop;
  }

  public void stop() {
    m_climberMover.setVoltage(0.0);
  }

  public boolean getHasFinishedCalibrating() {
    return hasFinishedCalibrating;
  }

  public void setHasFinishedCalibrating(boolean hasFinishedCalibrating) {
    this.hasFinishedCalibrating = hasFinishedCalibrating;
  }

  public boolean atSetpoint() {
    return Math.abs(positionSetpoint - m_climberRelativeEncoder.getPosition())
        < ClimberConstants.kAllowableError;
  }

  public CANSparkMax getMotorController() {
    return m_climberMover;
  }
}
