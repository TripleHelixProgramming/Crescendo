// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.Arm;
import frc.robot.drivetrain.Drivetrain;

public class ZorroDriveCommand extends DriveCommand {

  Arm m_arm;
  Joystick m_controller;

  public ZorroDriveCommand(Drivetrain subsystem, Arm arm, Joystick joysticks) {
    super(subsystem);
    this.m_arm = arm;
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

  @Override
  public Translation2d getSteeringCenter() {
    // offset steering angle whenever arm is raised
    // return m_arm.isArmRaised().getAsBoolean()
    //   ? ArmConstants.kChassisCentroidToArmCentroid
    //   : null;

    // offset steering angle when Zorro A button is pressed
    return m_controller.getRawButton(OIConstants.kZorroAIn)
        ? ArmConstants.kChassisCentroidToArmCentroid
        : null;
  }
}
