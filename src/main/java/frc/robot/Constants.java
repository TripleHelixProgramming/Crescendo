// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {

    // Define the conventional order of our modules when putting them into arrays
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int REAR_LEFT = 2;
    public static final int REAR_RIGHT = 3;

    public static final class MotorControllers {
      public static final int kRearRightDriveMotorPort = 12;
      public static final int kFrontRightDriveMotorPort = 26;
      public static final int kFrontLeftDriveMotorPort = 28;
      public static final int kRearLeftDriveMotorPort = 10;

      public static final int kRearRightTurningMotorPort = 13;
      public static final int kFrontRightTurningMotorPort = 27;
      public static final int kFrontLeftTurningMotorPort = 29;
      public static final int kRearLeftTurningMotorPort = 11;
    }

    public static final class AbsoluteEncoders {
      public static final int kRearRightTurningEncoderPort = 31;
      public static final int kFrontRightTurningEncoderPort = 33;
      public static final int kFrontLeftTurningEncoderPort = 43;
      public static final int kRearLeftTurningEncoderPort = 45;

      public static final String kAbsEncoderMagnetOffsetKey = "AbsEncoderMagnetOffsetKey";
      public static final double kDefaultAbsEncoderMagnetOffset = 0.0;
    }

    /**
     * public static final boolean kFrontLeftDriveEncoderReversed = false; public static final
     * boolean kFrontRightDriveEncoderReversed = false; public static final boolean
     * kRearLeftDriveEncoderReversed = true; public static final boolean
     * kRearRightDriveEncoderReversed = true;
     *
     * <p>public static final boolean kFrontLeftTurningEncoderReversed = false; public static final
     * boolean kFrontRightTurningEncoderReversed = false; public static final boolean
     * kRearLeftTurningEncoderReversed = true; public static final boolean
     * kRearRightTurningEncoderReversed = true;
     */

    // Units are meters.
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.572; // 2023 Competion Robot

    // Distance between front and rear wheels on robot
    public static final double kWheelBase = 0.622; // 2023 Competion Robot

    // Units are meters per second
    public static final double kMaxTranslationalVelocity = 0.73; // 2023 Competion Robot // max 4.5
    // public static final double kMaxSpeedMetersPerSecond = 4.0; // 2023 Competion Robot

    // Units are radians per second
    public static final double kMaxRotationalVelocity = 5.0; // 2023 Competion Robot // max 5.0

    // The locations for the modules must be relative to the center of the robot.
    // Positive x values represent moving toward the front of the robot
    // Positive y values represent moving toward the left of the robot.
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // rear left
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // rear right
            );

    // public static final boolean kGyroReversed = false;

    // public static final double ksVolts = 1.0;
    // public static final double kvVoltSecondsPerMeter = 0.8;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.15;
  }

  public static final class ModuleConstants {

    public static final double kDriveP = 0.1; // 2023 Competion Robot
    public static final double kDriveI = 0.0; // 2023 Competion Robot
    public static final double kDriveD = 0.0; // 2023 Competion Robot
    public static final double kDriveFF = 0.255; // 2023 Competion Robot

    public static final double kTurningP = 1.5; // 2023 Competion Robot
    public static final double kTurningI = 0.0; // 2023 Competion Robot
    public static final double kTurningD = 0.0; // 2023 Competion Robot

    // Not adjusted
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0.000005 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared =
        0.000005 * Math.PI;

    // public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254,
    // 0.137);

    public static final double kWheelDiameterMeters = 0.102; // 3.7 in; 2023 Competion Robot

    // By default, the drive encoder in position mode measures rotations at the drive motor
    // Convert to meters at the wheel
    public static final double kDriveGearRatio = 6.75; // 2023 Competion Robot
    public static final double kDrivePositionConversionFactor =
        (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // By default, the drive encoder in velocity mode measures RPM at the drive motor
    // Convert to meters per second at the wheel
    public static final double kDriveVelocityConversionFactor =
        kDrivePositionConversionFactor / 60.0;

    // By default, the turn encoder in position mode measures rotations at the turning motor
    // Convert to radians at the module azimuth
    public static final double kTurnGearRatio = 12.8; // 2023 Competion Robot
    public static final double kTurnPositionConversionFactor = (2.0 * Math.PI / kTurnGearRatio);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
