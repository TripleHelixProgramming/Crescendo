package frc.robot.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final ClimberSide m_leftClimberSide =
      new ClimberSide("Left", ClimberConstants.kLeftMotorPort);
  private final ClimberSide m_rightClimberSide =
      new ClimberSide("Right", ClimberConstants.kRightMotorPort);

  private ClimberSide[] m_actuators = {m_leftClimberSide, m_rightClimberSide};

  DifferentialDrive m_differentialDrive;

  public Climber() {
    m_differentialDrive =
        new DifferentialDrive(m_leftClimberSide::setPower, m_rightClimberSide::setPower);
  }

  /**
   * @return Vector of climber actuators
   */
  public ClimberSide[] getClimberSides() {
    return m_actuators;
  }

  public Command createStopCommand() {
    return this.runOnce(
        () -> {
          for (ClimberSide actuator : m_actuators) actuator.stop();
        });
  }

  public Command createArcadeDriveCommand(XboxController xboxController) {
    return this.run(
        () ->
            this.m_differentialDrive.arcadeDrive(
                -xboxController.getRightY(), -xboxController.getLeftX()));
  }

  @Override
  public void periodic() {
    for (ClimberSide actuator : m_actuators) actuator.periodic();
  }
}
