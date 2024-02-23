package frc.robot.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import java.util.function.BooleanSupplier;

public class Arm extends SubsystemBase {
  private ArmState m_armState;
  private final DoubleSolenoid m_armDeployer;
  private final DoubleSolenoid m_armHardStopper;

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
  }

  private ArmState getState() {
    return this.m_armState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Arm State", m_armState.name());
    
  }

  public BooleanSupplier stateChecker(ArmState state) {
    return () -> this.getState().equals(state);
  }

  private void setState(ArmState state) {
    this.m_armState = state;
  }

  public Command createStowCommand() {
    return this.runOnce(
        () -> {
          this.setState(ArmState.STOWED);
          this.m_armHardStopper.set(Value.kReverse);
          this.m_armDeployer.set(Value.kReverse);
        });
  }

  public Command createDeployCommand() {
    return this.runOnce(
        () -> {
          this.setState(ArmState.DEPLOYED);
          this.m_armHardStopper.set(Value.kReverse);
          this.m_armDeployer.set(Value.kForward);
        });
  }

  public Command createCarryCommand() {
    return this.runOnce(
        () -> {
          this.setState(ArmState.CARRY);
          this.m_armHardStopper.set(Value.kForward);
          this.m_armDeployer.set(Value.kReverse);
        });
  }
}
