// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.drivetrain.Drivetrain;

public abstract class DriveCommand extends Command {

  private double xDot;
  private double yDot;
  private double thetaDot;

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
    xDot = getX() * DriveConstants.kMaxTranslationalVelocity;
    yDot = getY() * DriveConstants.kMaxTranslationalVelocity;
    thetaDot = getTheta() * DriveConstants.kMaxRotationalVelocity;

    SmartDashboard.putBoolean("fieldRelative", getDriveMode() == DriveMode.FIELD_CENTRIC);
    SmartDashboard.putBoolean("rotateField", drivetrain.fieldRotatedSupplier().getAsBoolean());

    drivetrain.setChassisSpeeds(getChassisSpeeds(), kinematicsType);
  }

  private ChassisSpeeds getChassisSpeeds() {
    switch (getDriveMode()) {
      case FIELD_CENTRIC:
        var robotAngle = drivetrain.getHeading();
        if (drivetrain.fieldRotatedSupplier().getAsBoolean()) robotAngle.rotateBy(new Rotation2d(Math.PI));
        return ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, robotAngle);

      case ROBOT_CENTRIC_AFT_FACING:
        var rotated = new Translation2d(xDot, yDot).rotateBy(new Rotation2d(Math.PI));
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), thetaDot);

      case ROBOT_CENTRIC_FORWARD_FACING:
      default:
        return new ChassisSpeeds(xDot, yDot, thetaDot);
    }
  }

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
   * @return The desired drive mode
   */
  public abstract DriveMode getDriveMode();
}
