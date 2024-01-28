// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.Alliance;
import frc.robot.Constants.OIConstants;
import frc.robot.drivetrain.Drivetrain;

public class ZorroDrive extends Drive {

  Joystick m_controller;
  private Alliance alliance;

  public ZorroDrive(Drivetrain subsystem, Joystick joysticks, Alliance alliance) {
    super(subsystem);
    this.m_controller = joysticks;
    this.alliance = alliance;
  }

  @Override
  public double getX() {
    // return -m_controller.getRawAxis(OIConstants.kZorroRightYAxis);
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightYAxis), 0.05);
  }

  @Override
  public double getY() {
    // return -m_controller.getRawAxis(OIConstants.kZorroRightXAxis);
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroRightXAxis), 0.05);
  }

  @Override
  public double getTheta() {
    // return -m_controller.getRawAxis(OIConstants.kZorroLeftXAxis);
    return -MathUtil.applyDeadband(m_controller.getRawAxis(OIConstants.kZorroLeftXAxis), 0.05);
  }

  @Override
  public boolean getFieldRelative() {
    return m_controller.getRawButton(OIConstants.kZorroEUp);
  }

  @Override
  public Alliance getAlliance() {
    return alliance;
  }
}
