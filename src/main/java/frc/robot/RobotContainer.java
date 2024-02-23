// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LEDs.LEDs;
import frc.robot.arm.Arm;
import frc.robot.climber.Climber;
import frc.robot.climber.commands.CalibrateCommand;
import frc.robot.climber.commands.DriveToPositionCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;
import frc.robot.intake.Intake;
import java.util.function.IntSupplier;

public class RobotContainer {

  private static RobotContainer INSTANCE = null;

  public static RobotContainer getRobotContainer() {
    if (INSTANCE == null) {
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }

  private final PowerDistribution m_PowerDistribution = new PowerDistribution(1, ModuleType.kRev);

  private final Drivetrain m_swerve = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final LEDs m_LEDs = new LEDs();

  private final EventLoop m_loop = new EventLoop();
  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);
  private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  // digital inputs for autonomous selection
  private final DigitalInput[] autonomousModes =
      new DigitalInput[AutoConstants.kAutonomousModeSelectorPorts.length];

  private Autonomous m_autonomous;

  public RobotContainer() {

    for (int i = 0; i < AutoConstants.kAutonomousModeSelectorPorts.length; i++) {
      autonomousModes[i] = new DigitalInput(AutoConstants.kAutonomousModeSelectorPorts[i]);
    }

    createNamedCommands();
    setDefaultCommands();

    m_swerve.configurePathPlanner();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    configureDriverButtonBindings();
    configureOperatorButtonBindings();
  }

  /**
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    updateSelectedAutonomous();
    if (m_autonomous != null) return m_autonomous.getPathPlannerAuto();
    else return null;
  }

  // spotless:off
  public void teleopInit() {
    m_arm.createStowCommand().schedule();
    m_LEDs.createEnabledCommand(
    m_intake.eitherSensorSupplier(), m_arm.stateChecker(ArmState.DEPLOYED)).schedule();
  }

  public void autonomousInit() {
    m_LEDs.createEnabledCommand(
      m_intake.eitherSensorSupplier(), m_arm.stateChecker(ArmState.DEPLOYED)).schedule();
  }

  public void disabledInit() {
    m_LEDs.createDisabledCommand(m_swerve.redAllianceSupplier(), autonomousModeSelector()).schedule();
  }

    // spotless:on

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

  // spotless:off
  /** Updates the autonomous based on the physical selector switch */
  private void updateSelectedAutonomous() {
    switch (getAutonomousModeSwitchIndex()) {
      case 1:
        m_autonomous =
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? new Autonomous("R-TheOnePiece")
                : new Autonomous("B-TheOnePiece");
        break;

      case 2:
        m_autonomous =
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? null
                : new Autonomous("B-TheTwoPieceNear");
        break;

      case 3:
        m_autonomous =
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? null
                : new Autonomous("B-TwoPieceFar1");
        break;

      case 4:
        m_autonomous = 
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? null 
                : null;
        break;

      case 5:
        m_autonomous = 
            m_swerve.redAllianceSupplier().getAsBoolean()
                ? null 
                : new Autonomous("B-ThreePieceAutoKachow");
            break;

      default:
        m_autonomous = null;
    }
  }
  // spotless:on

  /**
   * @return Index in array of Digital Inputs corresponding to selected auto mode
   */
  private int getAutonomousModeSwitchIndex() {
    for (int port = 0; port < autonomousModes.length; port++) {
      if (!autonomousModes[port].get()) {
        return port + 1;
      }
    }
    return 0; // failure of the physical switch
  }

  private IntSupplier autonomousModeSelector() {
    return () -> getAutonomousModeSwitchIndex();
  }

  /**
   * Gets the current drawn from the Power Distribution Hub by a CAN motor controller, assuming that
   * (PDH port number + 10) = CAN ID
   *
   * @param CANBusPort The CAN ID of the motor controller
   * @return Current in Amps on the PDH channel corresponding to the motor channel
   */
  public double getPDHCurrent(int CANBusPort) {
    return m_PowerDistribution.getCurrent(CANBusPort - 10);
  }

  // spotless:off
  private void createNamedCommands() {

    NamedCommands.registerCommand("raiseArmAndWait", 

      m_arm.createDeployCommand()
        .andThen(new WaitCommand(1.4)));
    
    NamedCommands.registerCommand("resetArmAndIntake", 
      m_arm.createStowCommand()
        .alongWith(m_intake.createStopIntakeCommand()));
    
    NamedCommands.registerCommand("outtakeAndWait", 
      m_intake.createOuttakeToAmpCommand()
        .withTimeout(0.7));
    
    NamedCommands.registerCommand("intakePieceAndRaise", 
      createIntakeCommandSequence()
        .andThen(m_arm.createCarryCommand())
        .andThen(new WaitCommand(1.9)));
    
    NamedCommands.registerCommand("stopIntake", 
      m_intake.createStopIntakeCommand());
  }
  // spotless:on

  private void setDefaultCommands() {
    m_swerve.setDefaultCommand(
        new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematics, m_driver));
    m_intake.setDefaultCommand(m_intake.createStopIntakeCommand());
    m_climber.setDefaultCommand(m_climber.createStopCommand());
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    new JoystickButton(m_driver, OIConstants.kZorroHIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading())
        .ignoringDisable(true));

    new JoystickButton(m_driver,OIConstants.kZorroAIn)
    .whileTrue((new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematicsDriveFromArm, m_driver)));

    new JoystickButton(m_driver, OIConstants.kZorroDIn)
    .whileTrue(m_intake.createOuttakeToAmpCommand()
    .onlyIf(m_arm.stateChecker(ArmState.DEPLOYED)));
  }
  // spotless:on

  // spotless:off
  private void configureOperatorButtonBindings() {

    // CLIMBER
    // Calibrate upper limit of climber actuators
    new JoystickButton(m_operator, Button.kStart.value).onTrue(new CalibrateCommand(m_climber)
        .andThen(new DriveToPositionCommand(m_climber, ClimberConstants.kHomePosition)));

    // Deploy climber and begin climbing
    BooleanEvent climbThreshold = m_operator.axisGreaterThan(Axis.kRightY.value, -0.9, m_loop).debounce(0.1);
    Trigger climbTrigger = climbThreshold.castTo(Trigger::new);
    climbTrigger.onTrue(new DriveToPositionCommand(m_climber, ClimberConstants.kDeployPosition)
        .andThen(m_climber.createArcadeDriveCommand(m_operator)));

    // Move climber to home position
    // new JoystickButton(m_operator,Button.kB.value)
    //     .onTrue(m_climber.createDriveToCommand(ClimberConstants.kHomePosition));
    
    // Run climber drive while B button down
    // new JoystickButton(m_operator,Button.kB.value)
    //     .whileTrue(m_climber.createArcadeDriveCommand(m_operator));

    // INTAKE
    // Control position of Note in intake
    Trigger intakeTrigger = new Trigger(() -> Math.abs(m_operator.getLeftY()) > 0.2);
    intakeTrigger.whileTrue(m_intake.createIntakeJoystickControlCommand(m_operator));

    // Intake Note from floor
    JoystickButton intakeButton = new JoystickButton(m_operator, Button.kRightBumper.value);
    intakeButton
        // .whileTrue(createIntakeCommandSequence());
        .onTrue(m_arm.createStowCommand()
        .andThen(m_intake.createIntakeCommand().until(intakeButton.negate())));

    intakeButton
        .onFalse(m_intake.createStopIntakeCommand());

    JoystickButton leftBumper = new JoystickButton(m_operator, Button.kLeftBumper.value);
    Trigger deployed = new Trigger(m_arm.stateChecker(ArmState.DEPLOYED));
    
    // Reverse intake to outake or reject intaking Note
    leftBumper.and(deployed.negate())
        .whileTrue(m_intake.createOuttakeToFloorCommand());
    
    // Shoot Note into Amp
    leftBumper.and(deployed)
        .whileTrue(m_intake.createOuttakeToAmpCommand());


    
    // Shift Note further into Intake
    // new JoystickButton(m_operator, Button.kX.value)
    //     .onTrue(m_intake.createSetPositionCommand(0.05));


    // Move Note back in order to place in trap
    // new JoystickButton(m_operator, Button.kB.value)
    //     .whileTrue(m_intake.createSetPositionCommand(-0.27));

    // ARM
    // Raise and lower arm
    new JoystickButton(m_operator, Button.kA.value).onTrue(m_arm.createStowCommand());
    new JoystickButton(m_operator, Button.kY.value).onTrue(m_arm.createDeployCommand());
    
    // Deploy flap
    new POVButton(m_operator, OIConstants.kUp)
        .onTrue(m_arm.createFlapDeployCommand());
    // only while arm is raised

    // Stow flap
    new POVButton(m_operator, OIConstants.kDown)
        .onTrue(m_arm.createFlapRetractCommand());
    // only while arm is raised

    // MULTIPLE SUBSYSTEMS
    // Give Note to teammates
    new JoystickButton(m_operator, Button.kBack.value)
        .onTrue(m_arm.createDeployCommand()
          .alongWith(new WaitCommand(0.8))
        .andThen(m_intake.createOuttakeToFloorCommand()
          .raceWith(new WaitCommand(0.8)))
        .andThen(m_intake.createStopIntakeCommand()
          .alongWith(m_arm.createStowCommand())));
  }
  // spotless:on

  public Command createIntakeCommandSequence() {
    return new SequentialCommandGroup(
        m_arm.createStowCommand(),
        m_intake.createIntakeCommand().until(m_intake.eitherSensorSupplier()),
        m_arm.createCarryCommand(),
        m_intake
            .createAdvanceAfterIntakingCommand()
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }
}
