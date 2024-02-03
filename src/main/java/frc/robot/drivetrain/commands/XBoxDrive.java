// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drivetrain.Drivetrain;

public class XBoxDrive extends Drive {

  XboxController m_controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public XBoxDrive(Drivetrain subsystem, XboxController joysticks, boolean rotateField) {
    super(subsystem);
    this.m_controller = joysticks;
  }

  @Override
  public double getX() {
    return -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02));
  }

  @Override
  public double getY() {
    return -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02));
  }

  @Override
  public double getTheta() {
    return -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02));
  }

  @Override
  public boolean fieldRelative() {
    return m_controller.getLeftBumper();
  }
}
