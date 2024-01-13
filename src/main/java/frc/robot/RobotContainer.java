package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();

  private final XboxController m_controller = new XboxController(0);

  public RobotContainer() {

    m_swerve.setDefaultCommand(new JoystickDrive(m_swerve, m_controller));

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.setAbsTurningEncoderZero()).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = null;
    return autoCommand;
  }
}
