package frc.robot.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.CalibrationState;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;

public class ClimberSide {

  public final String climberName;

  private final CANSparkMax m_climberMover;
  private final RelativeEncoder m_climberRelativeEncoder;

  private final ProfiledPIDController m_climberPIDController =
      new ProfiledPIDController(
          ClimberConstants.kP,
          ClimberConstants.kI,
          ClimberConstants.kD,
          ClimberConstants.rapidConstraints);

  private LinearFilter m_filter = LinearFilter.singlePoleIIR(0.3, RobotConstants.kPeriod);
  private Debouncer m_debouncer = new Debouncer(0.05, DebounceType.kRising);

  private CalibrationState m_calibrationState = CalibrationState.UNCALIBRATED;

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

    m_climberPIDController.setTolerance(ClimberConstants.kAllowablePositionError);

    m_climberRelativeEncoder = m_climberMover.getEncoder();

    m_climberRelativeEncoder.setPositionConversionFactor(
        ClimberConstants.kPositionConversionFactor);
    m_climberRelativeEncoder.setVelocityConversionFactor(
        ClimberConstants.kVelocityConversionFactor);
  }

  public void configurePositionController(
      TrapezoidProfile.Constraints constraints, double targetPosition) {
    m_climberPIDController.setConstraints(constraints);
    m_climberPIDController.setGoal(targetPosition);
    m_climberPIDController.reset(m_climberRelativeEncoder.getPosition());
  }

  public void driveToTargetPosition() {
    m_climberMover.set(m_climberPIDController.calculate(getPosition()));
  }

  public void setPower(double power) {
    m_climberMover.set(power);
  }

  public void stop() {
    m_climberMover.set(0.0);
  }

  /**
   * @param upperLimitEnabled Whether the upper soft limit should be enabled
   */
  public void configureUpperLimit(boolean upperLimitEnabled) {
    m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLimitEnabled);
  }

  public void resetEncoder() {
    m_climberRelativeEncoder.setPosition(0.0);
  }

  public boolean getCurrentSenseState() {
    return m_debouncer.calculate(
        m_filter.calculate(m_climberMover.getOutputCurrent())
            > ClimberConstants.kMotorCurrentHardStop);
  }

  private boolean getUpperSoftLimitSwtichState() {
    return m_climberMover.getFault(CANSparkBase.FaultID.kSoftLimitFwd);
  }

  private boolean getLowerSoftLimitSwtichState() {
    return m_climberMover.getFault(CANSparkBase.FaultID.kSoftLimitRev);
  }

  /**
   * @return State of the actuator calibration
   */
  public CalibrationState getCalibrationState() {
    return m_calibrationState;
  }

  /**
   * @param calibrationState State of the actuator calibration
   */
  public void setCalibrationState(CalibrationState calibrationState) {
    this.m_calibrationState = calibrationState;
  }

  /**
   * @return True when climber actuator is within allowable error of setpoint of closed-loop
   *     position controller
   */
  public boolean atGoal() {
    return m_climberPIDController.atGoal();
  }

  /**
   * @return Position of climber actuator in inches
   */
  private double getPosition() {
    return m_climberRelativeEncoder.getPosition();
  }

  /**
   * @return Current in Amps, output of linear filter
   */
  private double getCurrent() {
    return m_filter.calculate(m_climberMover.getOutputCurrent());
  }

  /**
   * @return Name of climber actuator
   */
  private String getName() {
    return "Climber" + climberName;
  }

  public void periodic() {
    SmartDashboard.putNumber(getName() + "Stroke", getPosition());
    SmartDashboard.putNumber(
        getName() + "MotorRotations", getPosition() / ClimberConstants.kPositionConversionFactor);
    SmartDashboard.putBoolean(getName() + "UpperSoftLimitState", getUpperSoftLimitSwtichState());
    SmartDashboard.putBoolean(getName() + "LowerSoftLimitState", getLowerSoftLimitSwtichState());
    SmartDashboard.putNumber(getName() + "SparkMaxCurrent", addPolarity(getCurrent()));
    // SmartDashboard.putNumber(getName() + "PDHCurrent", m_powerDistribtion ClimberConstants.kLeftMotorPort - 10);
    SmartDashboard.putString(getName() + "CalibrationState", getCalibrationState().name());
  }

  private double addPolarity(double value) {
    return value * Math.signum(m_climberMover.getAppliedOutput());
  }
}
