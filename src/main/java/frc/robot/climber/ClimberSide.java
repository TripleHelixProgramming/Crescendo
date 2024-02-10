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

public class ClimberSide extends SubsystemBase {

    private final CANSparkMax m_climberMover;
    private final RelativeEncoder m_climberRelativeEncoder;
    private final SparkPIDController m_climberPIDController;

    DifferentialDrive m_arcadeDrive;

    public ClimberSide(int climberMotorChannel) {
        m_climberMover = new CANSparkMax(climberMotorChannel, MotorType.kBrushless);

        m_climberMover.restoreFactoryDefaults();

        m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_climberMover.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_climberMover.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ClimberConstants.kUpperLimit);
        m_climberMover.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ClimberConstants.kLowerLimit);

        m_climberMover.setIdleMode(IdleMode.kBrake);
        m_climberMover.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
        m_climberMover.setInverted(false);

        m_climberPIDController = m_climberMover.getPIDController();

        m_climberPIDController.setP(ClimberConstants.kClimberP);
        m_climberPIDController.setI(ClimberConstants.kClimberI);
        m_climberPIDController.setD(ClimberConstants.kClimberD);

        m_climberRelativeEncoder = m_climberMover.getEncoder();

        m_climberRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversionFactor);
        m_climberRelativeEncoder.setVelocityConversionFactor(ClimberConstants.kClimberVelocityConversionFactor);
    }

    private void setVelocity(double targetVelocity) {
        m_climberPIDController.setReference(targetVelocity, ControlType.kVelocity);
    }

    private void setVoltage(double targetVoltage) {
        m_climberPIDController.setReference(targetVoltage, ControlType.kVoltage);
    }

    private void setPosition(double targetPosition) {
        m_climberPIDController.setReference(targetPosition, ControlType.kPosition);
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

    public void stopMotor() {
        m_climberMover.setVoltage(0.0);
    }

    public void setMotorVoltage(double targetVoltage) {
        m_climberPIDController.setReference(targetVoltage, ControlType.kVoltage);
    }

    public Command createStopCommand() {
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

    public Command createResetEncodersCommand() {
        return this.runOnce(() -> this.resetEncoder());
    }

    public Command createConfigureUpperLimitCommand(boolean upperLImitEnabled) {
        return this.runOnce(() -> this.configureUpperLimit(upperLImitEnabled));
    }
}