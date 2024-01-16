// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain;

import static frc.robot.Constants.OIConstants;

import edu.wpi.first.wpilibj.Joystick;

public class ZorroDrive extends Drive {

  Joystick m_controller;

  public ZorroDrive(Drivetrain subsystem, Joystick joysticks) {
    super(subsystem);
    this.m_controller = joysticks;
  }

  @Override
  public double getX() {
    return -m_controller.getRawAxis(OIConstants.kZorroRightYAxis);
  }

  @Override
  public double getY() {
    return -m_controller.getRawAxis(OIConstants.kZorroRightXAxis);
  }

  @Override
  public double getTheta() {
    return -m_controller.getRawAxis(OIConstants.kZorroLeftXAxis);
  }

  @Override
  public boolean getFieldRelative() {
    return m_controller.getRawButton(OIConstants.kZorroEUp);
  }
}
