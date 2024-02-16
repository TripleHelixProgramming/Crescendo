package frc.robot.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private final Actuator[] m_actuators = new Actuator[2];
  private final DifferentialDrive m_differentialDrive;

  public Climber() {
    m_actuators[0] = new Actuator("Left", ClimberConstants.kLeftMotorPort);
    m_actuators[1] = new Actuator("Right", ClimberConstants.kRightMotorPort);
    m_differentialDrive = new DifferentialDrive(m_actuators[0]::setPower, m_actuators[1]::setPower);
  }

  /**
   * @return Vector of climber actuators
   */
  public Actuator[] getClimberSides() {
    return m_actuators;
  }

  public Command createStopCommand() {
    return this.runOnce(
        () -> {
          for (Actuator actuator : m_actuators) actuator.stop();
        });
  }

  public Command createArcadeDriveCommand(XboxController xboxController) {
    return this.run(
        () ->
            this.m_differentialDrive.arcadeDrive(
                -xboxController.getRightY(), -xboxController.getLeftX()));
  }

  public Command createDriveToPositionCommand(double targetPosition) {
    return new FunctionalCommand(
        // initialize
        () -> {
          for (Actuator actuator : m_actuators) {
            actuator.configurePositionController(ClimberConstants.rapidConstraints, targetPosition);
          }
        },
        // execute
        () -> {
          for (Actuator actuator : m_actuators) actuator.driveToTargetPosition();
        },
        // end
        interrupted -> {},
        // isFinished
        () -> {
          for (Actuator actuator : m_actuators) if (!actuator.atGoal()) return false;
          return true;
        },
        // requirements
        this);
  }

  @Override
  public void periodic() {
    m_differentialDrive.feed();
    for (Actuator actuator : m_actuators) actuator.periodic();
  }
}
