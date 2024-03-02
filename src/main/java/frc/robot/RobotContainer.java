// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.ZorroDriveCommand;

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

  private final EventLoop m_loop = new EventLoop();
  private Joystick m_driver = new Joystick(OIConstants.kDriverControllerPort);

  private Autonomous m_autonomous;

  public RobotContainer() {

    createNamedCommands();
    setDefaultCommands();

    m_swerve.configurePathPlanner();

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.zeroAbsTurningEncoderOffsets()).ignoringDisable(true));

    configureDriverButtonBindings();
  }

  /**
   * @return The Command that runs the selected autonomous mode
   */
  public Command getAutonomousCommand() {
    updateSelectedAutonomous();
    if (m_autonomous != null) return m_autonomous.getPathPlannerAuto();
    else return null;
  }

  public Command createTeleopInitSequence() {
    return new SequentialCommandGroup();
  }

  public Command createAutonomousInitSequence() {
    return new SequentialCommandGroup();
  }

  public Command createDisabledInitSequence() {
    return new SequentialCommandGroup();
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
    m_autonomous = new Autonomous("B-driveFwd3m");
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

  private void createNamedCommands() {}

  private void setDefaultCommands() {
    m_swerve.setDefaultCommand(
        new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematics, m_driver));
  }

  // spotless:off
  private void configureDriverButtonBindings() {

    // Reset heading
    new JoystickButton(m_driver, OIConstants.kZorroHIn)
        .onTrue(new InstantCommand(() -> m_swerve.resetHeading())
        .ignoringDisable(true));

    new JoystickButton(m_driver,OIConstants.kZorroAIn)
    .whileTrue((new ZorroDriveCommand(m_swerve, DriveConstants.kDriveKinematicsDriveFromArm, m_driver)));
  }
  // spotless:on

}
