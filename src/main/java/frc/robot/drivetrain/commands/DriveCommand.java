// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.Drivetrain;

public abstract class DriveCommand extends Command {

  private Measure<Velocity<Distance>> xDot;
  private Measure<Velocity<Distance>> yDot;
  private Measure<Velocity<Angle>> thetaDot;

  // used to swap control locations
  SwerveDriveKinematics kinematicsType;

  // The subsystem the command runs on
  public final Drivetrain drivetrain;

  public DriveCommand(Drivetrain subsystem, SwerveDriveKinematics kinematicsType) {
    drivetrain = subsystem;
    addRequirements(drivetrain);

    this.kinematicsType = kinematicsType;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xDot = DriveConstants.kMaxTranslationalVelocity.times(getX());
    yDot = DriveConstants.kMaxTranslationalVelocity.times(getY());
    thetaDot = DriveConstants.kMaxRotationalVelocity.times(getTheta());

    SmartDashboard.putBoolean("fieldRelative", fieldRelative());
    SmartDashboard.putBoolean("rotateField", drivetrain.fieldRotatedSupplier().getAsBoolean());

    drivetrain.setChassisSpeeds(
        fieldRelative() ? getFieldRelativeChassisSpeeds() : getRobotRelativeChassisSpeeds(),
        kinematicsType);
  }

  private ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return new ChassisSpeeds(xDot, yDot, thetaDot);
  }

  // spotless:off
  private ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return drivetrain.fieldRotatedSupplier().getAsBoolean()
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xDot, yDot, thetaDot, drivetrain.getHeading().rotateBy(new Rotation2d(Math.PI)))
        : ChassisSpeeds.fromFieldRelativeSpeeds(
            xDot, yDot, thetaDot, drivetrain.getHeading());
  }
  // spotless:on

  /**
   * @return The input to the drive controller in the x axis, range [-1, 1]
   */
  public abstract double getX();

  /**
   * @return The input to the drive controller in the y axis, range [-1, 1]
   */
  public abstract double getY();

  /**
   * @return The input to the drive controller in the theta axis, range [-1, 1]
   */
  public abstract double getTheta();

  /**
   * @return Boolean representing whether to drive in field-relative mode
   */
  public abstract boolean fieldRelative();
}
