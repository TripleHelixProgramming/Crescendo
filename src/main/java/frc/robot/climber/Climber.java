package frc.robot.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final ClimberSide m_leftClimberSide =
      new ClimberSide("Left", ClimberConstants.kLeftMotorPort);
  private final ClimberSide m_rightClimberSide =
      new ClimberSide("Right", ClimberConstants.kRightMotorPort);

  private ClimberSide[] climberSides = {m_leftClimberSide, m_rightClimberSide};

  DifferentialDrive m_differentialDrive;

  public Climber() {
    m_differentialDrive =
        new DifferentialDrive(m_leftClimberSide::setPower, m_rightClimberSide::setPower);
  }

  /**
   * @return Vector of climber actuators
   */
  public ClimberSide[] getClimberSides() {
    return climberSides;
  }

  public Command createStopCommand() {
    return this.runOnce(
        () -> {
          m_leftClimberSide.stop();
          m_rightClimberSide.stop();
        });
  }

  public Command createDriveToCommand(double targetPosition) {
    return this.run(
        () -> {
          m_leftClimberSide.driveRapidlyTo(targetPosition);
          m_rightClimberSide.driveRapidlyTo(targetPosition);
        });
  }

  public Command createArcadeDriveCommand(XboxController xboxController) {
    return this.run(
        () ->
            this.m_differentialDrive.arcadeDrive(
                -xboxController.getRightY(), -xboxController.getLeftX()));
  }

  /**
   * @return True when both climber actuators are within allowable error of setpoint of closed-loop
   *     position controller
   */
  public boolean bothSidesAtSetpoint() {
    return m_leftClimberSide.atGoal() && m_rightClimberSide.atGoal();
  }

  /**
   * @return True when both climber actuators are in the calibration-complete state
   */
  public boolean bothSidesCalibrated() {
    return m_leftClimberSide.getHasFinishedCalibrating()
        && m_rightClimberSide.getHasFinishedCalibrating();
  }

  // spotless:off
  @Override
  public void periodic() {
    for (ClimberSide climberside : climberSides) {
      SmartDashboard.putNumber("Climber" + climberside.getName() + 
        "Height", climberside.getHeight());
      SmartDashboard.putBoolean("Climber" + climberside.getName() + 
        "UpperSoftLimitState", climberside.getUpperSoftLimitSwtichDetected());
      SmartDashboard.putBoolean("Climber" + climberside.getName() + 
        "LowerSoftLimitState", climberside.getLowerSoftLimitSwtichDetected());
      SmartDashboard.putNumber("Climber" + climberside.getName() + 
        "Current", climberside.getCurrent());
    }
  }
  // spotless:on
}
