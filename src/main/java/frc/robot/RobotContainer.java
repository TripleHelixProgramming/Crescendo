package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();

  private final XboxController m_controller = new XboxController(0);

  public RobotContainer() {

    m_swerve.setDefaultCommand(new JoystickDrive(m_swerve, m_controller, true));

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    new JoystickButton(m_controller, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_swerve.resetGyro()).ignoringDisable(true));

    new JoystickButton(m_controller, Button.kLeftBumper.value)
        .whileTrue(new JoystickDrive(m_swerve, m_controller, false));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = null;
    return autoCommand;
  }
}
