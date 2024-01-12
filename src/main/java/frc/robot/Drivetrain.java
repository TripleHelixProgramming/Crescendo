// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  static double kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static double kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.SparkCAN.kFrontLeftDriveMotorPort,
          DriveConstants.SparkCAN.kFrontLeftTurningMotorPort,
          DriveConstants.CANCoder.kFrontLeftTurningEncoderPort,
          DriveConstants.CANCoder.kFrontLeftTurningEncoderOffset);
  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.SparkCAN.kFrontRightDriveMotorPort,
          DriveConstants.SparkCAN.kFrontRightTurningMotorPort,
          DriveConstants.CANCoder.kFrontRightTurningEncoderPort,
          DriveConstants.CANCoder.kFrontRightTurningEncoderOffset);
  private final SwerveModule m_backLeft =
      new SwerveModule(
          DriveConstants.SparkCAN.kRearLeftDriveMotorPort,
          DriveConstants.SparkCAN.kRearLeftTurningMotorPort,
          DriveConstants.CANCoder.kRearLeftTurningEncoderPort,
          DriveConstants.CANCoder.kRearLeftTurningEncoderOffset);
  private final SwerveModule m_backRight =
      new SwerveModule(
          DriveConstants.SparkCAN.kRearRightDriveMotorPort,
          DriveConstants.SparkCAN.kRearRightTurningMotorPort,
          DriveConstants.CANCoder.kRearRightTurningEncoderPort,
          DriveConstants.CANCoder.kRearRightTurningEncoderOffset);

  private final AHRS m_gyro = new AHRS();

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
