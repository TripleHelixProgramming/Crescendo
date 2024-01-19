package frc.robot.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotorA;
  private final CANSparkMax m_intakeMotorB;

  private final RelativeEncoder m_intakeRelativeEncoderA;
  private final RelativeEncoder m_intakeRelativeEncoderB;

  private final SparkPIDController m_intakePIDControllerA;
  private final SparkPIDController m_intakePIDControllerB;

  public final DigitalInput m_noteSensor;

  public Intake() {
    m_intakeMotorA = new CANSparkMax(ArmConstants.k_intakeMotorAPort, MotorType.kBrushless);
    m_intakeMotorB = new CANSparkMax(ArmConstants.k_intakeMotorBPort, MotorType.kBrushless);

    m_intakeMotorA.restoreFactoryDefaults();
    m_intakeMotorB.restoreFactoryDefaults();

    m_intakeMotorA.setIdleMode(IdleMode.kBrake);
    m_intakeMotorB.setIdleMode(IdleMode.kBrake);

    m_intakeMotorA.setSmartCurrentLimit(ArmConstants.k_intakeMotorCurrentLimit);
    m_intakeMotorB.setSmartCurrentLimit(ArmConstants.k_intakeMotorCurrentLimit);

    m_intakeMotorA.setInverted(false);
    m_intakeMotorB.setInverted(false);

    m_intakePIDControllerA = m_intakeMotorA.getPIDController();
    m_intakePIDControllerB = m_intakeMotorB.getPIDController();

    m_intakePIDControllerA.setP(ArmConstants.kIntakeP);
    m_intakePIDControllerA.setI(ArmConstants.kIntakeI);
    m_intakePIDControllerA.setD(ArmConstants.kIntakeD);

    m_intakePIDControllerB.setP(ArmConstants.kIntakeP);
    m_intakePIDControllerB.setI(ArmConstants.kIntakeI);
    m_intakePIDControllerB.setD(ArmConstants.kIntakeD);

    m_intakeRelativeEncoderA = m_intakeMotorA.getEncoder();
    m_intakeRelativeEncoderB = m_intakeMotorB.getEncoder();

    m_intakeRelativeEncoderA.setPositionConversionFactor(
        ArmConstants.kIntakePositionConversionFactor);
    m_intakeRelativeEncoderA.setVelocityConversionFactor(
        ArmConstants.kIntakeVelocityConversionFactor);

    m_intakeRelativeEncoderB.setPositionConversionFactor(
        ArmConstants.kIntakePositionConversionFactor);
    m_intakeRelativeEncoderB.setVelocityConversionFactor(
        ArmConstants.kIntakeVelocityConversionFactor);

    m_noteSensor = new DigitalInput(ArmConstants.kNoteSensorDIOPort);
  }

  public void setPosition(double targetPosition) {
    m_intakePIDControllerA.setReference(targetPosition, ControlType.kPosition);
    m_intakePIDControllerB.setReference(targetPosition, ControlType.kPosition);
  }

  public void setVelocity(double targetVelocity) {
    m_intakePIDControllerA.setReference(targetVelocity, ControlType.kVelocity);
    m_intakePIDControllerB.setReference(targetVelocity, ControlType.kVelocity);
  }

  public void resetIntakeEncoder() {
    m_intakeRelativeEncoderA.setPosition(0.0);
    m_intakeRelativeEncoderB.setPosition(0.0);
  }

  public boolean hasGamePiece() {
    return m_noteSensor.get();
  }
}
