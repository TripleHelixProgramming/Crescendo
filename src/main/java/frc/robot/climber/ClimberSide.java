package frc.robot.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotConstants;

public class ClimberSide {

  public final String climberName;

  private final CANSparkMax m_climberMover;
  private final RelativeEncoder m_climberRelativeEncoder;
  // private final SparkPIDController m_climberPIDController;
  private final ProfiledPIDController m_climberPIDController =
      new ProfiledPIDController(
          ClimberConstants.kP,
          ClimberConstants.kI,
          ClimberConstants.kD,
          ClimberConstants.rapidConstraints);

  private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, RobotConstants.kPeriod);

  private boolean hasFinishedCalibrating = false;

  public ClimberSide(String climberName, int climberMotorChannel) {

    this.climberName = climberName;

    m_climberMover = new CANSparkMax(climberMotorChannel, MotorType.kBrushless);

    m_climberMover.restoreFactoryDefaults();

    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    m_climberMover.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.kUpperLimit);
    m_climberMover.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.kLowerLimit);

    m_climberMover.setIdleMode(IdleMode.kBrake);
    m_climberMover.setSmartCurrentLimit(ClimberConstants.kMotorCurrentLimit);
    m_climberMover.setInverted(false);

    m_climberPIDController.setTolerance(ClimberConstants.kAllowableError);

    // m_climberPIDController = m_climberMover.getPIDController();

    // m_climberPIDController.setP(ClimberConstants.kP);
    // m_climberPIDController.setI(ClimberConstants.kI);
    // m_climberPIDController.setD(ClimberConstants.kD);

    m_climberRelativeEncoder = m_climberMover.getEncoder();

    m_climberRelativeEncoder.setPositionConversionFactor(
        ClimberConstants.kPositionConversionFactor);
    m_climberRelativeEncoder.setVelocityConversionFactor(
        ClimberConstants.kVelocityConversionFactor);
  }

  // public void setVelocity(double targetVelocity) {
  //   m_climberPIDController.setReference(targetVelocity, ControlType.kVelocity);
  // }

  public void setVoltage(double voltage) {
    m_climberMover.setVoltage(voltage);
  }

  public void driveRapidlyTo(double targetPosition) {
    m_climberPIDController.setConstraints(ClimberConstants.rapidConstraints);
    m_climberPIDController.setGoal(targetPosition);
    drive();
  }

  public void driveSlowlyTo(double targetPosition) {
    m_climberPIDController.setConstraints(ClimberConstants.slowConstraints);
    m_climberPIDController.setGoal(targetPosition);
    drive();
  }

  public void drive() {
    m_climberMover.setVoltage(
        m_climberPIDController.calculate(m_climberRelativeEncoder.getPosition()));
  }

  public void setPower(double power) {
    m_climberMover.set(power);
  }

  public void configureUpperLimit(boolean upperLImitEnabled) {
    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLImitEnabled);
  }

  public void resetEncoder() {
    m_climberRelativeEncoder.setPosition(0.0);
  }

  public boolean getUpperHardStopDetected() {
    return filter.calculate(m_climberRelativeEncoder.getPosition())
        > ClimberConstants.kMotorCurrentHardStop;
  }

  public boolean getUpperSoftLimitSwtichDetected() {
    return m_climberMover.getFault(CANSparkBase.FaultID.kSoftLimitFwd);
  }

  public boolean getLowerSoftLimitSwtichDetected() {
    return m_climberMover.getFault(CANSparkBase.FaultID.kSoftLimitRev);
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

  public boolean atGoal() {
    return m_climberPIDController.atGoal();
  }

  public double getHeight() {
    return m_climberRelativeEncoder.getPosition();
  }

  public double getClimberCurrent() {
    return m_climberMover.getOutputCurrent();
  }

  public String getName() {
    return climberName;
  }
}
