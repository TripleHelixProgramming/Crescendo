package frc.robot.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;

  private final RelativeEncoder m_relativeEncoder;

  private final SparkPIDController m_velocityController;

  private final ProfiledPIDController m_positionController =
      new ProfiledPIDController(
          IntakeConstants.kPositionP,
          IntakeConstants.kPositionI,
          IntakeConstants.kPositionD,
          IntakeConstants.kConstraints);

  private final DigitalInput m_noteSensorBeamBreak =
      new DigitalInput(IntakeConstants.kBeamBreakSensorDIOPort);
  private final DigitalInput m_noteSensorRetroReflective =
      new DigitalInput(IntakeConstants.kRetroReflectiveSensorDIOPort);

  private final EventLoop m_loop = new EventLoop();
  private final BooleanEvent m_RRsensorTriggered =
      new BooleanEvent(m_loop, retroReflectiveSensorSupplier());

  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();

    m_intakeMotor.setIdleMode(IdleMode.kBrake);

    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);

    m_intakeMotor.setInverted(false);

    m_velocityController = m_intakeMotor.getPIDController();
    m_velocityController.setP(IntakeConstants.kVelocityP);
    m_velocityController.setI(IntakeConstants.kVelocityI);
    m_velocityController.setD(IntakeConstants.kVelocityD);

    m_positionController.setTolerance(IntakeConstants.kPositionTolerance);

    m_relativeEncoder = m_intakeMotor.getEncoder();
    m_relativeEncoder.setPosition(0.0);
    m_relativeEncoder.setPositionConversionFactor(IntakeConstants.kPositionConversionFactor);
    m_relativeEncoder.setVelocityConversionFactor(IntakeConstants.kVelocityConversionFactor);
  }

  private void stopIntake() {
    m_intakeMotor.set(0.0);
  }

  public Command createStopIntakeCommand() {
    return this.runOnce(() -> this.stopIntake());
  }

  private void configurePositionController(double targetPosition) {
    m_positionController.reset(m_relativeEncoder.getPosition(), m_relativeEncoder.getVelocity());
    m_positionController.setGoal(m_relativeEncoder.getPosition() + targetPosition);
  }

  private void advanceAfterIntaking(double targetPosition) {
    m_RRsensorTriggered
        .rising()
        .ifHigh(
            () -> {
              m_positionController.reset(
                  m_relativeEncoder.getPosition(), m_relativeEncoder.getVelocity());
              m_positionController.setGoal(m_relativeEncoder.getPosition() + targetPosition);
            });
    m_intakeMotor.set(m_positionController.calculate(m_relativeEncoder.getPosition()));
  }

  private void driveToPosition() {
    m_intakeMotor.set(m_positionController.calculate(m_relativeEncoder.getPosition()));
  }



  public Command createAdvanceAfterIntakingCommand() {
    return new FunctionalCommand(
        // initialize
        () -> this.configurePositionController(IntakeConstants.kRepositionAfterIntaking),
        // execute
        () -> this.advanceAfterIntaking(IntakeConstants.kRepositionAfterIntaking),
        // end
        interrupted -> {},
        // isFinished
        this.atGoalSupplier(),
        // requirements
        this);
  }

  public Command createSetPositionCommand(double targetPosition) {
    return new FunctionalCommand(
        // initialize
        () -> this.configurePositionController(targetPosition),
        // execute
        () -> this.driveToPosition(),
        // end
        interrupted -> {},
        // isFinished
        this.atGoalSupplier(),
        // requirements
        this);
  }

  private void setVelocity(double targetVelocity) {
    m_velocityController.setReference(targetVelocity, ControlType.kVelocity);
  }

  public Command createSetVelocityCommand(double targetVelocity) {
    return this.startEnd(() -> this.setVelocity(targetVelocity), () -> {});
  }

  private void setVoltage(double targetVoltage) {
    m_velocityController.setReference(targetVoltage, ControlType.kVoltage);
  }

  public Command createSetVoltageCommand(double targetVoltage) {
    // /return this.startEnd(() -> this.setVoltage(targetVoltage), () -> {});
    return this.run(() -> this.setVoltage(targetVoltage));
  }

  public BooleanSupplier eitherSensorSupplier() {
    return () -> (!m_noteSensorRetroReflective.get() || !m_noteSensorBeamBreak.get());
  }

  public BooleanSupplier retroReflectiveSensorSupplier() {
    return () -> !m_noteSensorRetroReflective.get();
  }

  public BooleanSupplier atGoalSupplier() {
    return () -> m_positionController.atGoal();
  }

  @Override
  public void periodic() {
    m_loop.poll();

    SmartDashboard.putNumber("OutputCurrent", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("IntakePosition", m_relativeEncoder.getPosition());
    SmartDashboard.putNumber("IntakeGoal", m_positionController.getGoal().position);
    SmartDashboard.putNumber(
        "IntakeSensorRetroReflective", !m_noteSensorRetroReflective.get() ? 1d : 0d);
    SmartDashboard.putNumber("IntakeSensorBeamBreak", !m_noteSensorBeamBreak.get() ? 1d : 0d);
  }
}
