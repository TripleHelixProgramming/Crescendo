// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.Constants.Alliance;
import frc.robot.Constants.AutoConstants.Auto;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.Arm;
import frc.robot.climber.CalibrateCommand;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDrive;
import frc.robot.intake.Intake;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  private final EventLoop m_loop = new EventLoop();
  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);
  private XboxController m_operator = new XboxController(OIConstants.kOperatorControllerPort);

  private Auto m_selectedAuto;

  // spotless:off
  public RobotContainer() {
    m_selectedAuto = Auto.B_DRIVEFWD2M;

    m_swerve.setDefaultCommand(new ZorroDrive(m_swerve, m_driver, getAlliance()));
    m_swerve.configurePathPlanner();

    m_intake.setDefaultCommand(m_intake.createStopIntakeCommand());
    m_climber.setDefaultCommand(m_climber.createStopCommand());

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData("Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets())
            .ignoringDisable(true));

    // Driver controller buttons
    new JoystickButton(m_driver, OIConstants.kZorroDIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetGyro())
            .ignoringDisable(true));

    // Operator controller buttons

    // Calibrate upper limit of climber actuators
    new JoystickButton(m_operator, Button.kStart.value).onTrue(new CalibrateCommand(m_climber)
    .andThen(m_climber.createSetPositionCommand(ClimberConstants.kHomePosition)));

    // Deploy climber and begin climbing
    BooleanEvent climbThreshold = m_operator.axisGreaterThan(Axis.kRightY.value, 0.75, m_loop).debounce(0.1);
    Trigger climbTrigger = climbThreshold.castTo(Trigger::new);
    climbTrigger.onTrue(m_climber.createSetPositionCommand(ClimberConstants.kDeployPosition)
        .until(m_climber::bothSidesAtSetpoint)
        .andThen(m_climber.createArcadeDriveCommand(m_operator)));

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
    // new JoystickButton(m_operator, Button.kB.value)
    //     .whileTrue(m_intake.createSetVoltageCommand(-12.0));

    // Moves note back in order to place in trap
    new JoystickButton(m_operator, Button.kB.value)
        .whileTrue(m_intake.createSetPositionCommand(-0.27));

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

  public Command getAutonomousCommand() {
    switch (m_selectedAuto) {
      case B_DRIVEFWD2M:
        return new PathPlannerAuto("B-driveFwd2m");
      case R_DRIVEFWD2M:
        return new PathPlannerAuto("R-driveFwd2m");
      case B_DRIVERIGHTTURNTORIGHT:
        return new PathPlannerAuto("B_SpinForward");
      default:
        return null;
    }
  }

  public Alliance getAlliance() {
    switch (m_selectedAuto) {
      case R_DRIVEFWD2M:
        return Alliance.RED_ALLIANCE;
      case B_DRIVEFWD2M:
      case B_DRIVERIGHTTURNTORIGHT:
      default:
        return Alliance.BLUE_ALLIANCE;
    }
  }

  public void teleopInit() {
    m_arm.createLowerArmCommand().schedule();
  }

  public void periodic() {
    SmartDashboard.putString("Alliance", getAlliance().toString());
    m_loop.poll();
  }
}
