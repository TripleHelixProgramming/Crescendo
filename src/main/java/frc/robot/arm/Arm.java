package frc.robot.arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final DoubleSolenoid m_armMoverLeft;
  private final DoubleSolenoid m_armMoverRight;

  public Arm() {
    m_armMoverLeft =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kArmMoverLeftForwardChannel,
            ArmConstants.kArmMoverLeftReverseChannel);
    m_armMoverRight =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kArmMoverRightForwardChannel,
            ArmConstants.kArmMoverRightReverseChannel);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish the arm state to telemetry.
    builder.addBooleanProperty(
        "ArmRaised",
        () -> (m_armMoverLeft.get() == Value.kForward) && (m_armMoverRight.get() == Value.kForward),
        null);
  }

  public void pneumaticDeploy() {
    m_armMoverLeft.set(Value.kForward);
    m_armMoverRight.set(Value.kForward);
  }

  public void pneumaticRetract() {
    m_armMoverLeft.set(Value.kReverse);
    m_armMoverRight.set(Value.kReverse);
  }

  // Command Factory methods
  // See
  // https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#instance-command-factory-methods

  public Command lowerArmCommand() {
    return this.startEnd(() -> this.pneumaticRetract(), () -> {});
  }

  public Command raiseArmCommand() {
    return this.startEnd(() -> this.pneumaticDeploy(), () -> {});
  }
}
