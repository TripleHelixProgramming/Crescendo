package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_leftClimberMover;
    private final CANSparkMax m_rightClimberMover;

    private final RelativeEncoder m_leftClimberRelativeEncoder;
    private final RelativeEncoder m_rightClimberRelativeEncoder;

    private final SparkPIDController m_leftClimberPIDController;
    private final SparkPIDController m_rightClimberPIDController;

    DifferentialDrive m_arcadeDrive;

    public Climber() {
        m_leftClimberMover = new CANSparkMax(ClimberConstants.kLeftClimberMotorPort, MotorType.kBrushless);
        m_rightClimberMover = new CANSparkMax(ClimberConstants.kRightClimberMotorPort, MotorType.kBrushless);

        m_leftClimberMover.restoreFactoryDefaults();
        m_rightClimberMover.restoreFactoryDefaults();

        m_leftClimberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_rightClimberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_leftClimberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_rightClimberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_leftClimberMover.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.kUpperLimit);
        m_rightClimberMover.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.kUpperLimit);
        m_leftClimberMover.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.kLowerLimit);
        m_rightClimberMover.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.kLowerLimit);

        m_leftClimberMover.setIdleMode(IdleMode.kBrake);
        m_rightClimberMover.setIdleMode(IdleMode.kBrake);

        m_leftClimberMover.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
        m_rightClimberMover.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);

        m_leftClimberMover.setInverted(false);
        m_rightClimberMover.setInverted(false);

        m_leftClimberPIDController = m_leftClimberMover.getPIDController();
        m_rightClimberPIDController = m_rightClimberMover.getPIDController();

        m_leftClimberPIDController.setP(ClimberConstants.kClimberP);
        m_leftClimberPIDController.setI(ClimberConstants.kClimberI);
        m_leftClimberPIDController.setD(ClimberConstants.kClimberD);

        m_rightClimberPIDController.setP(ClimberConstants.kClimberP);
        m_rightClimberPIDController.setI(ClimberConstants.kClimberI);
        m_rightClimberPIDController.setD(ClimberConstants.kClimberD);

        m_leftClimberRelativeEncoder = m_leftClimberMover.getEncoder();
        m_rightClimberRelativeEncoder = m_rightClimberMover.getEncoder();

        m_leftClimberRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversionFactor);
        m_rightClimberRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversionFactor);

        m_leftClimberRelativeEncoder.setVelocityConversionFactor(ClimberConstants.kClimberVelocityConversionFactor);
        m_rightClimberRelativeEncoder.setVelocityConversionFactor(ClimberConstants.kClimberVelocityConversionFactor);

        m_arcadeDrive = new DifferentialDrive(m_leftClimberMover, m_rightClimberMover);
    }

    private void setVelocity(double targetVelocity) {
        m_leftClimberPIDController.setReference(targetVelocity, ControlType.kVelocity);
        m_rightClimberPIDController.setReference(targetVelocity, ControlType.kVelocity);
    }

    private void setVoltage(double targetVoltage) {
        m_leftClimberPIDController.setReference(targetVoltage, ControlType.kVoltage);
        m_rightClimberPIDController.setReference(targetVoltage, ControlType.kVoltage);
    }

    private void setPosition(double targetPosition) {
        m_leftClimberPIDController.setReference(targetPosition, ControlType.kPosition);
        m_rightClimberPIDController.setReference(targetPosition, ControlType.kPosition);
    }

    private void arcadeDrive(XboxController xboxController) {
        m_arcadeDrive.arcadeDrive(-xboxController.getRightY(), -xboxController.getLeftX());
    }

    public void configureUpperLimit(boolean upperLImitEnabled) {
        m_leftClimberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLImitEnabled);
        m_rightClimberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperLImitEnabled);
    }

    public void resetEncoders() {
        m_leftClimberRelativeEncoder.setPosition(0.0);
        m_rightClimberRelativeEncoder.setPosition(0.0);
    }

    public boolean getUpperLimitDetected(CANSparkMax climberMotor) {
        return climberMotor.getOutputCurrent() > ClimberConstants.kClimberMotorCurrentHardStop;
    }

    public void stopMotor(CANSparkMax climberMotor) {
        climberMotor.setVoltage(0.0);
    }

    public void setMotorVoltage(SparkPIDController climberPIDController, double targetVoltage) {
        climberPIDController.setReference(targetVoltage, ControlType.kVoltage);
    }

    public CANSparkMax[] getMotors() {
        CANSparkMax[] motors = {m_leftClimberMover, m_rightClimberMover};
        return motors;
    }

    public SparkPIDController[] getPIDControllers() {
        SparkPIDController[] PIDControllers = {m_leftClimberPIDController, m_rightClimberPIDController};
        return PIDControllers;
    }

    public Command createClimberStopCommand() {
        return this.runOnce(() -> this.setVoltage(0.0));
    }

    public Command createSetVelocityCommand(XboxController xboxController) {
        return this.run(() -> this.setVelocity(xboxController.getRightY()));
    }

    public Command createSetVoltageCommand(XboxController xboxController) {
        return this.run(() -> this.setVoltage(xboxController.getRightY()));
    }

    public Command createSetPositionCommand(double targetPosition) {
        return this.runOnce(() -> this.setPosition(targetPosition));
    }

    public Command createArcadeDriveCommand(XboxController xboxController) {
        return this.run(() -> this.arcadeDrive(xboxController));
    }

    public Command createResetEncodersCommand() {
        return this.runOnce(() -> this.resetEncoders());
    }

    public Command createConfigureUpperLimitCommand(boolean upperLImitEnabled) {
        return this.runOnce(() -> this.configureUpperLimit(upperLImitEnabled));
    }
}