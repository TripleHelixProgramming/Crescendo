// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

// import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ZorroDrive;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();

  private final Arm m_arm = new Arm();

  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);

  private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  // private final XboxController m_controller = new
  // XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver));

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetGyro()).ignoringDisable(true));

    new JoystickButton(m_operator, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.pneumaticRetract()));

    new JoystickButton(m_operator, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.pneumaticDeploy()));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = null;
    return autoCommand;
  }
}
