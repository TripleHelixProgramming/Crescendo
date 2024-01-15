package frc.robot;

// import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ZorroDrive;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();

  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);

  // private final XboxController m_controller = new
  // XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver, true));

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    new JoystickButton(m_driver, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_swerve.resetGyro()).ignoringDisable(true));

    new JoystickButton(m_driver, Button.kLeftBumper.value)
        .whileTrue(new ZorroDrive(m_swerve, m_driver, false));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = null;
    return autoCommand;
  }
}
