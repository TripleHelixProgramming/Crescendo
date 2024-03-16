// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
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
<<<<<<< HEAD
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightYAxis), 0.05);
=======
    return m_controller.getRawButton(OIConstants.kZorroEDown)
        ? -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightYAxis), 0.05) * -1
        : -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightYAxis), 0.05);
>>>>>>> a58795856102ede730e08de9da27f1a03699f233
  }

  @Override
  public double getY() {
<<<<<<< HEAD
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightXAxis), 0.05);
=======
    return m_controller.getRawButton(OIConstants.kZorroEDown)
        ? -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightXAxis), 0.05) * -1
        : -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightXAxis), 0.05);
>>>>>>> a58795856102ede730e08de9da27f1a03699f233
  }

  @Override
  public double getTheta() {
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroLeftXAxis), 0.05);
  }

  @Override
  public boolean fieldRelative() {
    return m_controller.getRawButton(OIConstants.kZorroEUp);
  }
}
