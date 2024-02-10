package frc.robot.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final ClimberSide m_leftClimberSide =
      new ClimberSide(ClimberConstants.kLeftClimberMotorPort);
  private final ClimberSide m_rightClimberSide =
      new ClimberSide(ClimberConstants.kRightClimberMotorPort);

  private ClimberSide[] climberSides = {m_leftClimberSide, m_rightClimberSide};

  DifferentialDrive m_arcadeDrive;

  public Climber() {

    m_arcadeDrive =
        new DifferentialDrive(
            m_leftClimberSide.getMotorController(), m_rightClimberSide.getMotorController());
  }

  public ClimberSide[] getClimberSides() {
    return climberSides;
  }

  public void stop() {
    for (ClimberSide climberSide : climberSides) {
      climberSide.stop();
    }
  }

  public Command createStopCommand() {
    return this.runOnce(() -> this.stop());
  }

  public Command createSetPositionCommand(double targetPosition) {
    return this.runOnce(
        () -> {
          m_leftClimberSide.setPositionSetpoint(targetPosition);
          m_rightClimberSide.setPositionSetpoint(targetPosition);
          m_leftClimberSide.setPosition();
          m_rightClimberSide.setPosition();
        });
  }

  private void arcadeDrive(XboxController xboxController) {
    m_arcadeDrive.arcadeDrive(-xboxController.getRightY(), -xboxController.getLeftX());
  }

  public Command createArcadeDriveCommand(XboxController xboxController) {
    return this.run(() -> this.arcadeDrive(xboxController));
  }

  public boolean bothSidesAtSetpoint() {
    return m_leftClimberSide.atSetpoint() && m_rightClimberSide.atSetpoint();
  }
}
