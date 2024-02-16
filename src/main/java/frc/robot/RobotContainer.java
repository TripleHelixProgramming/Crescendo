// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.Arm;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.CalibrateCommand;
import frc.robot.climber.commands.DriveToPositionCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.intake.Intake;

public class RobotContainer {

  private final PowerDistribution m_PowerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private final Drivetrain m_swerve = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber(m_PowerDistribution);

  private final EventLoop m_loop = new EventLoop();
  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);
  private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  

  // digital inputs for autonomous selection
  private final DigitalInput[] autonomousModes =
      new DigitalInput[AutoConstants.kAutonomousModeSelectorPorts.length];

  private Autonomous m_autonomous;

  // spotless:off
  public RobotContainer() {

    for (int i = 0; i < AutoConstants.kAutonomousModeSelectorPorts.length; i++) {
      autonomousModes[i] = new DigitalInput(AutoConstants.kAutonomousModeSelectorPorts[i]);
    }

    m_swerve.setDefaultCommand(new ZorroDriveCommand(m_swerve, m_driver));
    m_swerve.configurePathPlanner();

    m_intake.setDefaultCommand(m_intake.createStopIntakeCommand());
    m_climber.setDefaultCommand(m_climber.createStopCommand());

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData("Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets())
        .ignoringDisable(true));

    // Driver controller buttons
    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading())
        .ignoringDisable(true));

    // Operator controller buttons

    // Calibrate upper limit of climber actuators
    new JoystickButton(m_operator, Button.kStart.value).onTrue(new CalibrateCommand(m_climber)
        .andThen(new DriveToPositionCommand(m_climber, ClimberConstants.kHomePosition)));

    // Deploy climber and begin climbing
    BooleanEvent climbThreshold = m_operator.axisGreaterThan(Axis.kRightY.value, -0.9, m_loop).debounce(0.1);
    Trigger climbTrigger = climbThreshold.castTo(Trigger::new);
    climbTrigger.onTrue(new DriveToPositionCommand(m_climber, ClimberConstants.kDeployPosition)
        .andThen(m_climber.createArcadeDriveCommand(m_operator)));
    
    //Run climber drive while B button down
    // new JoystickButton(m_operator,Button.kB.value)
    // .whileTrue(m_climber.createDriveToCommand(ClimberConstants.kHomePosition));
    // .whileTrue(m_climber.createArcadeDriveCommand(m_operator));
    
    // new JoystickButton(m_operator,Button.kB.value)
    //     .whileTrue(m_climber.createArcadeDriveCommand(m_operator));

    // Raise and lower arm
    new JoystickButton(m_operator, Button.kA.value).onTrue(m_arm.createLowerArmCommand());
    new JoystickButton(m_operator, Button.kY.value).onTrue(m_arm.createRaiseArmCommand());

    // Intake Note from floor
    new JoystickButton(m_operator, Button.kRightBumper.value)
        .whileTrue(m_intake.createSetVoltageCommand(12.0)
        .until(m_intake::hasGamePiece)
        .andThen(m_intake.createSetPositionCommand(0.2))
        .onlyIf(m_arm.isArmLowered()));

    // Shift Note further into Intake
    new JoystickButton(m_operator, Button.kX.value)
        .whileTrue(m_intake.createSetPositionCommand(0.25));

    // Shoot Note into Amp
    new JoystickButton(m_operator, Button.kLeftBumper.value)
        .whileTrue(m_intake.createSetVoltageCommand(12.0)
        .onlyIf(m_arm.isArmRaised()));

    // Reverses intake
    new JoystickButton(m_operator, Button.kB.value)
        .whileTrue(m_intake.createSetVoltageCommand(-12.0));

    // Moves note back in order to place in trap
    // new JoystickButton(m_operator, Button.kB.value)
    //     .whileTrue(m_intake.createSetPositionCommand(-0.27));

    // Gives note to teammates
    new JoystickButton(m_operator, Button.kBack.value)
        .onTrue(m_arm.createRaiseArmCommand()
          .alongWith(new WaitCommand(0.8))
        .andThen(m_intake.createSetVoltageCommand(-12)
          .raceWith(new WaitCommand(0.8)))
        .andThen(m_intake.createStopIntakeCommand()
          .alongWith(m_arm.createLowerArmCommand())));
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
    m_arm.createLowerArmCommand().schedule();
  }

  public void periodic() {
    m_loop.poll();
    updateSelectedAutonomous();

    if (m_autonomous != null) {
      SmartDashboard.putString("Auto", m_autonomous.getFilename());
    } else {
      SmartDashboard.putString("Auto", "Null");
    }

    SmartDashboard.putNumber("PDHVoltage", m_PowerDistribution.getVoltage());
    SmartDashboard.putNumber("PDHTotalCurrent", m_PowerDistribution.getTotalCurrent());
  }

  private class Autonomous {

    private final String filename;

    private Autonomous(String filename) {
      this.filename = filename;
    }

    private Command getPathPlannerAuto() {
      return new PathPlannerAuto(filename);
    }

    private String getFilename() {
      return filename;
    }
  }

  /** Updates the autonomous based on the physical selector switch */
  private void updateSelectedAutonomous() {
    switch (getSelectedAutonomousMode()) {
      case 0:
        m_autonomous =
            m_swerve.getRedAlliance()
                ? new Autonomous("R-driveFwd2m")
                : new Autonomous("B-driveFwd2m");
        break;

      case 1:
        m_autonomous =
            m_swerve.getRedAlliance()
                ? new Autonomous("R-driveFwd2m")
                : new Autonomous("B_SpinForward");
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
