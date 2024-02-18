package frc.robot.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;

  private final RelativeEncoder m_intakeRelativeEncoder;

  private final SparkPIDController m_intakePIDController;

  private final ProfiledPIDController m_intakeProfiledPIDController = 
  new ProfiledPIDController(ArmConstants.kIntakePositionP, 
                            ArmConstants.kIntakePositionI,
                            ArmConstants.kIntakePositionD, 
                            ArmConstants.intakeConstraints);

  private final DigitalInput m_noteSensor = new DigitalInput(ArmConstants.kNoteSensorDIOPort);

  public Intake() {
    m_intakeMotor = new CANSparkMax(ArmConstants.k_intakeMotorPort, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();

    m_intakeMotor.setIdleMode(IdleMode.kBrake);

    m_intakeMotor.setSmartCurrentLimit(ArmConstants.k_intakeMotorCurrentLimit);

    m_intakeMotor.setInverted(false);

    // m_intakeMotor
    //     .getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
    //     .enableLimitSwitch(false);

    m_intakePIDController = m_intakeMotor.getPIDController();

    m_intakeRelativeEncoder = m_intakeMotor.getEncoder();

    m_intakeProfiledPIDController.setTolerance(ArmConstants.kIntakeTolerance);

    m_intakeRelativeEncoder.setPositionConversionFactor(
        ArmConstants.kIntakePositionConversionFactor);
    m_intakeRelativeEncoder.setVelocityConversionFactor(
        ArmConstants.kIntakeVelocityConversionFactor);
  }

  private void stopIntake() {
    m_intakeMotor.setVoltage(0.0);
  }

  public Command createStopIntakeCommand() {
    return this.runOnce(() -> this.stopIntake());
  }

  private void setPosition(double targetPosition) {
    m_intakePIDController.setP(ArmConstants.kIntakePositionP);
    m_intakePIDController.setI(ArmConstants.kIntakePositionI);
    m_intakePIDController.setD(ArmConstants.kIntakePositionD);
    m_intakeRelativeEncoder.setPosition(0.0);
    m_intakePIDController.setReference(targetPosition, ControlType.kPosition);
  }

  public Command createSetPositionCommand(double targetPosition) {
    return this.startEnd(() -> this.setPosition(targetPosition), () -> {});
  }

  public void configurePositionController(double targetPosition){
    m_intakeProfiledPIDController.setGoal(targetPosition);
    m_intakeProfiledPIDController.reset(m_intakeRelativeEncoder.getPosition());
  }

  public void driveToTargetPosition(){
    m_intakeMotor.set(m_intakeProfiledPIDController.calculate(getPosition()));
  }

  private void setVelocity(double targetVelocity) {
    m_intakePIDController.setP(ArmConstants.kIntakeVelocityP);
    m_intakePIDController.setI(ArmConstants.kIntakeVelocityI);
    m_intakePIDController.setD(ArmConstants.kIntakeVelocityD);
    m_intakePIDController.setReference(targetVelocity, ControlType.kVelocity);
  }

  public Command createSetVelocityCommand(double targetVelocity) {
    return this.startEnd(() -> this.setVelocity(targetVelocity), () -> {});
  }

  private void setVoltage(double targetVoltage) {
    m_intakePIDController.setReference(targetVoltage, ControlType.kVoltage);
  }

  public Command createSetVoltageCommand(double targetVoltage) {
    // return this.startEnd(() -> this.setVoltage(targetVoltage), () -> {});
    return this.run(() -> this.setVoltage(targetVoltage));
  }

  public void resetIntakeEncoder() {
    m_intakeRelativeEncoder.setPosition(0.0);
  }

  public Command createResetEncoderCommand() {
    return this.runOnce(() -> this.resetIntakeEncoder());
  }

  public boolean hasGamePiece() {
    // return m_intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    return !m_noteSensor.get();
  }

  private double GamePieceDetected() {
    if (hasGamePiece() == true) {
      return 1.0;
    } else {
      return 0.0;
    }
  }

  private double getPosition(){
    return m_intakeRelativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("OutputCurrent", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("hasGamePiece", GamePieceDetected());
  }
}
