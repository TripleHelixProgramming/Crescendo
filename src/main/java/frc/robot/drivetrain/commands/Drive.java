// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.Drivetrain;

public abstract class Drive extends Command {

  private double xDot;
  private double yDot;
  private double thetaDot;
  private boolean fieldRelative;
  private Alliance alliance;
  private ChassisSpeeds chassisSpeeds;

  // The subsystem the command runs on
  public final Drivetrain drivetrain;

  public Drive(Drivetrain subsystem) {
    drivetrain = subsystem;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xDot = getX() * DriveConstants.kMaxTranslationalVelocity;
    yDot = getY() * DriveConstants.kMaxTranslationalVelocity;
    thetaDot = getTheta() * DriveConstants.kMaxRotationalVelocity;
    fieldRelative = getFieldRelative();
    alliance = getAlliance();

    chassisSpeeds =
        fieldRelative
            ? (alliance == Alliance.RED_ALLIANCE)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xDot, yDot, thetaDot, drivetrain.getHeading().unaryMinus())
                : ChassisSpeeds.fromFieldRelativeSpeeds(
                    xDot, yDot, thetaDot, drivetrain.getHeading())
            : new ChassisSpeeds(xDot, yDot, thetaDot);

    drivetrain.drive(chassisSpeeds);
  }

  public abstract double getX();

  public abstract double getY();

  public abstract double getTheta();

  public abstract boolean getFieldRelative();

  public abstract Alliance getAlliance();
}
