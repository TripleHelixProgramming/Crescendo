// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Alliance;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDrive;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();
  // private final Arm m_arm = new Arm();
  // private final Intake m_intake = new Intake();

  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);

  // private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  // spotless:off
  public RobotContainer() {

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver, getAlliance()));
    m_swerve.configurePathPlanner();

    // m_intake.setDefaultCommand(m_intake.createStopIntakeCommand());

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData("Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets())
            .ignoringDisable(true));

    // Driver controller buttons
    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading(getAlliance()))
            .ignoringDisable(true));

    // Command lowerArmCommand = m_arm.createLowerArmCommand();
    // Command raiseArmCommmand = m_arm.createRaiseArmCommand();
    // // Operator controller buttons
    // new JoystickButton(m_operator, Button.kLeftBumper.value).onTrue(lowerArmCommand);
    // new JoystickButton(m_operator, Button.kRightBumper.value).onTrue(raiseArmCommmand);

    // // Intake Note from floor
    // new JoystickButton(m_operator, Button.kX.value)
    //     .whileTrue((m_intake.createSetVoltageCommand(10.0)));
    //     // .until(m_intake::hasGamePiece));
    //     // .onlyIf(lowerArmCommand::isScheduled));

    // // Shift Note further into Intake
    // new JoystickButton(m_operator, Button.kA.value)
    //     .onTrue((m_intake.createResetEncoderCommand())
    //     .andThen(m_intake.createSetPositionCommand(0.2)));

    // // Shoot Note into Amp
    // new JoystickButton(m_operator, Button.kY.value)
    //     .whileTrue((m_intake.createSetVoltageCommand(10.0)));
    //     // .onlyIf(raiseArmCommmand::isScheduled));
  }
  // spotless:on

  /**
   * @return The selected autonomous mode
   */
  public Autonomous getSelectedAutonomous() {
    return new Autonomous("R-driveFwd2m", Alliance.RED_ALLIANCE);
    // return new Autonomous("B_SpinForward", Alliance.BLUE_ALLIANCE);
  }

  /**
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    return getSelectedAutonomous().getPathPlannerAuto();
  }

  /**
   * @return The alliance color corresponding to the selected autonomous mode
   */
  public Alliance getAlliance() {
    return getSelectedAutonomous().getAlliance();
  }

  public void teleopInit() {
    // m_arm.createLowerArmCommand().schedule();
  }

  public void periodic() {
    SmartDashboard.putString("Alliance", getAlliance().toString());
  }

  private class Autonomous {

    private final String filename;
    private final Alliance alliance;

    private Autonomous(String filename, Alliance alliance) {
      this.filename = filename;
      this.alliance = alliance;
    }

    private Command getPathPlannerAuto() {
      return new PathPlannerAuto(filename);
    }

    private Alliance getAlliance() {
      return alliance;
    }
  }
}
