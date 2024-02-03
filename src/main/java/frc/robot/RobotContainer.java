// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AllianceColor;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDrive;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();
  // private final Arm m_arm = new Arm();
  // private final Intake m_intake = new Intake();

  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);

  // private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  // digital inputs for autonomous selection
  private final DigitalInput[] autonomousModes = new DigitalInput[Constants.dioPortNumbers.length];
  private final DigitalInput allianceSelectionSwitch;

  private Autonomous m_autonomous;

  public RobotContainer() {

    for (int i = 0; i < Constants.dioPortNumbers.length; i++) {
      autonomousModes[i] = new DigitalInput(Constants.dioPortNumbers[i]);
    }

    allianceSelectionSwitch = new DigitalInput(Constants.dioAllianceSwitchPort);

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver));
    m_swerve.configurePathPlanner();

    // m_intake.setDefaultCommand(m_intake.createStopIntakeCommand());

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    // Driver controller buttons
    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading()).ignoringDisable(true));
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
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    updateSelectedAutonomous();
    return m_autonomous.getPathPlannerAuto();
  }

  public void teleopInit() {
    // m_arm.createLowerArmCommand().schedule();
  }

  public void periodic() {
    updateSelectedAutonomous();
    if (m_autonomous != null) {
      SmartDashboard.putString("Auto", m_autonomous.getFilename());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }
  }

  private class Autonomous {

    private final String filename;

    private Autonomous(String filename, AllianceColor alliance) {
      this.filename = filename;
    }

    private Command getPathPlannerAuto() {
      return new PathPlannerAuto(filename);
    }

    private String getFilename() {
      return filename;
    }
  }

  private void updateAllianceColor() {
    m_swerve.setAlliance(
        allianceSelectionSwitch.get() ? AllianceColor.RED_ALLIANCE : AllianceColor.BLUE_ALLIANCE);
  }

  /** Updates the autonomous based on the physical selector switch */
  private void updateSelectedAutonomous() {
    updateAllianceColor();

    switch (getSelectedAutonomousMode()) {
      case 0:
        m_autonomous =
            m_swerve.getRedAlliance()
                ? new Autonomous("R-driveFwd2m", AllianceColor.RED_ALLIANCE)
                : new Autonomous("B-driveFwd2m", AllianceColor.BLUE_ALLIANCE);
        break;

      case 1:
        m_autonomous =
            m_swerve.getRedAlliance()
                ? new Autonomous("R-driveFwd2m", AllianceColor.RED_ALLIANCE)
                : new Autonomous("B_SpinForward", AllianceColor.BLUE_ALLIANCE);
        break;

      case 2:

      case 3:

      case 4:

      case 5:

      case 6:

      case 7:

      default:
        m_autonomous = null;
    }
  }

  /**
   * @return Index in array of Digital Inputs corresponding to selected auto mode
   */
  private int getSelectedAutonomousMode() {
    for (int port = 0; port < autonomousModes.length; port++) {
      if (!autonomousModes[port].get()) {
        return port;
      }
    }
    return -1; // failure of the physical switch
  }
}
