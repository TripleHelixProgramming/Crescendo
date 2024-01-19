// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDrive;
import frc.robot.intake.Intake;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();

  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);
  private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  private StartEndCommand lowerArmCommand =
      new StartEndCommand(() -> m_arm.pneumaticRetract(), () -> {}, m_arm);

  private StartEndCommand raiseArmCommand =
      new StartEndCommand(() -> m_arm.pneumaticDeploy(), () -> {}, m_arm);

  public RobotContainer() {

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver));

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    // Driver controller buttons
    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetGyro()).ignoringDisable(true));

    // Operator controller buttons
    new JoystickButton(m_operator, Button.kLeftBumper.value).onTrue(lowerArmCommand);
    new JoystickButton(m_operator, Button.kRightBumper.value).onTrue(raiseArmCommand);

    // Intake Note from floor
    new JoystickButton(m_operator, Button.kX.value)
        .whileTrue(
            new RunCommand(() -> m_intake.setVelocity(1.0))
                .until(m_intake.m_noteSensor::get)
                .onlyIf(lowerArmCommand::isScheduled));

    // Shift Note further into Intake
    new JoystickButton(m_operator, Button.kA.value)
        .onTrue(
            new InstantCommand(() -> m_intake.resetIntakeEncoder())
                .andThen(new RunCommand(() -> m_intake.setPosition(0.2))));

    // Shoot Note into Amp
    new JoystickButton(m_operator, Button.kY.value)
        .whileTrue(
            new RunCommand(() -> m_intake.setVelocity(1.0)).onlyIf(raiseArmCommand::isScheduled));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = null;
    return autoCommand;
  }
}
