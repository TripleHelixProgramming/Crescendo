package frc.robot.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final ClimberSide m_leftClimberSide =
      new ClimberSide("left",ClimberConstants.kLeftMotorPort);
  private final ClimberSide m_rightClimberSide =
      new ClimberSide("right",ClimberConstants.kRightMotorPort);

  private ClimberSide[] climberSides = {m_leftClimberSide, m_rightClimberSide};

  DifferentialDrive m_differentialDrive;

  public Climber() {

    m_differentialDrive =
        new DifferentialDrive(
            m_leftClimberSide::setPower, m_rightClimberSide::setPower);
  }

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
          m_leftClimberSide.driveTo(targetPosition);
          m_rightClimberSide.driveTo(targetPosition);
        });
  }

  public Command createArcadeDriveCommand(XboxController xboxController) {
    return this.run(
        () ->
            this.m_differentialDrive.arcadeDrive(
                -xboxController.getRightY(), -xboxController.getLeftX()));
  }

  public boolean bothSidesAtSetpoint() {
    return m_leftClimberSide.atGoal() && m_rightClimberSide.atGoal();
  }

  @Override
  public void periodic(){
    for(ClimberSide climberside : climberSides){
      climberside.periodic();
      
      SmartDashboard.putNumber("Climber"+ climberside.climberName + " height", climberside.getHeight());
      SmartDashboard.putBoolean("Climber"+climberside.climberName + " atUpperLimit", climberside.getUpperSoftLimitSwtichDetected());
      SmartDashboard.putBoolean("Climber"+climberside.climberName + " atLowerLimit", climberside.getLowerSoftLimitSwtichDetected());
      SmartDashboard.putNumber("Climber"+climberside.climberName + " Current", climberside.getClimberCurrent());
    }
  }
}
