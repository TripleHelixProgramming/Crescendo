// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;

public class ZorroDriveCommand extends DriveCommand {

  Joystick m_controller;

  public ZorroDriveCommand(
      Drivetrain subsystem, SwerveDriveKinematics kinematicsType, Joystick joysticks) {
    super(subsystem, kinematicsType);
    this.m_controller = joysticks;
  }

  @Override
  public double getX() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightYAxis), 0.05);
  }

  @Override
  public double getY() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightXAxis), 0.05);
  }

  @Override
  public double getTheta() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroLeftXAxis), 0.05);
  }

  @Override
  public DriveMode getDriveMode() {
    DriveMode mode;

    if (m_controller.getRawButton(OIConstants.kZorroEUp)) mode = DriveMode.FIELD_CENTRIC;
    else if (m_controller.getRawButton(OIConstants.kZorroEDown))
      mode = DriveMode.ROBOT_CENTRIC_AFT_FACING;
    else mode = DriveMode.FIELD_CENTRIC;

    return mode;
  }
}
