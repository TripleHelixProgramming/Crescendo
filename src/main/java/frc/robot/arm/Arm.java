package frc.robot.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

  public void pneumaticDeploy() {
    m_armMoverLeft.set(Value.kForward);
    m_armMoverRight.set(Value.kForward);
  }

  public void pneumaticRetract() {
    m_armMoverLeft.set(Value.kReverse);
    m_armMoverRight.set(Value.kReverse);
  }
}
