// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoDrive extends Drive {

  ChassisSpeeds chasy;

  public AutoDrive(Drivetrain subsystem, ChassisSpeeds chasy) {
    super(subsystem);
    this.chasy = chasy;
  }

  @Override
  public double getX() {
    return chasy.vxMetersPerSecond;
  }

  @Override
  public double getY() {
    return chasy.vyMetersPerSecond;
  }

  @Override
  public double getTheta() {
    return chasy.omegaRadiansPerSecond;
  }

  @Override
  public boolean getFieldRelative() {
    return false;
  }
}
