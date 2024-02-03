// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;

public class ZorroDrive extends Drive {

  Joystick m_controller;

  public ZorroDrive(Drivetrain subsystem, Joystick joysticks) {
    super(subsystem);
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
  public boolean fieldRelative() {
    return m_controller.getRawButton(OIConstants.kZorroEUp);
  }
}
