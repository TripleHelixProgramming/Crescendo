// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  //private final Arm m_arm = new Arm();
  //private final Intake m_intake = new Intake();

  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);
  private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  // digital Inputs
  private final DigitalInput[] autoSwitches = new DigitalInput[Constants.dioPortNumbers.length];

  private final DigitalInput allianceSelectionSwitch;
  
  private Autonomous m_autonomous;

  public RobotContainer() {

    for(int i = 0; i < Constants.dioPortNumbers.length; i++){
      autoSwitches[i] = new DigitalInput(Constants.dioPortNumbers[i]);
    }    

    allianceSelectionSwitch = new DigitalInput(Constants.dioAllianceSwitchPort);
    

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver, getAlliance()));
    m_swerve.configurePathPlanner();

    //m_intake.setDefaultCommand(m_intake.createStopIntakeCommand());

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData("Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets())
            .ignoringDisable(true));

    // Driver controller buttons
    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetGyro())
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
  public void updateSelectedAutonomous() {

    int selectedSwitch = getSelectedDIO();

    switch (selectedSwitch) {
      case 0:
          m_autonomous = getAllianceSwitchIsBlue()
            ? new Autonomous("B-driveFwd2m", Alliance.BLUE_ALLIANCE)
            : new Autonomous("R-driveFwd2m", Alliance.RED_ALLIANCE);
            break;
      case 1:
          m_autonomous = getAllianceSwitchIsBlue()
            ? new Autonomous("B_SpinForward", Alliance.BLUE_ALLIANCE)
            : new Autonomous("R-driveFwd2m", Alliance.RED_ALLIANCE);

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

  public int getSelectedDIO() {

    for (int dioPort = 0; dioPort < autoSwitches.length; dioPort++) {
      if (!autoSwitches[dioPort].get()) {
        return dioPort;
      }
    }

    return -1;
  }

  public boolean getAllianceSwitchIsBlue() {
    return !allianceSelectionSwitch.get();
  }

  /**
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    updateSelectedAutonomous();
    return m_autonomous.getPathPlannerAuto();
    
    //return getSelectedAutonomous().getPathPlannerAuto();
  }

  /**
   * @return The alliance color corresponding to the selected autonomous mode
   */
  public Alliance getAlliance() {
    updateSelectedAutonomous();
    return m_autonomous.getAlliance();
  }

  public void teleopInit() {
    // m_arm.createLowerArmCommand().schedule();
  }

  public void periodic() {
    if(m_autonomous != null){
    SmartDashboard.putString("Alliance", getAlliance().toString());
    SmartDashboard.putString("Auto", m_autonomous.getFilename());
    }
    else{
      SmartDashboard.putString("Alliance", "Null");
      SmartDashboard.putString("Auto", "Null");
    }
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

    private String getFilename() {
      return filename;
    }
  }
}
