package frc.robot.arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import java.util.function.BooleanSupplier;

public class Arm extends SubsystemBase {
  private final DoubleSolenoid m_armDeployer;
  private final DoubleSolenoid m_armHardStopper;
  private final DoubleSolenoid m_flapRetracter;

  public Arm() {
    m_armDeployer =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kDeployerForwardChannel,
            ArmConstants.kDeployerReverseChannel);
    m_armHardStopper =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kHardStopperForwardChannel,
            ArmConstants.kHardStopperReverseChannel);
    m_flapRetracter =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ArmConstants.kFlapForwardChannel,
            ArmConstants.kFlapReverseChannel);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish the arm state to telemetry.
    builder.addBooleanProperty("ArmRaised", () -> m_armDeployer.get() == Value.kForward, null);
  }

  public Command createLowerArmCommand() {
    return this.runOnce(() -> this.m_armDeployer.set(Value.kReverse));
  }

  public Command createRaiseArmCommand() {
    return this.runOnce(() -> this.m_armDeployer.set(Value.kForward));
  }

  public Command createHardStopDeployCommand() {
    return this.runOnce(() -> this.m_armHardStopper.set(Value.kForward));
  }

  public Command createHardStopRetractCommand() {
    return this.runOnce(() -> this.m_armHardStopper.set(Value.kReverse));
  }

  public Command createFlapDeployCommand() {
    return this.runOnce(() -> this.m_flapRetracter.set(Value.kForward));
  }

  public Command createFapRetractCommand() {
    return this.runOnce(() -> this.m_flapRetracter.set(Value.kReverse));
  }

  public BooleanSupplier isArmRaised() {
    return () -> m_armDeployer.get().equals(Value.kForward);
  }

  public BooleanSupplier isArmLowered() {
    return () -> m_armDeployer.get().equals(Value.kReverse);
  }
}
