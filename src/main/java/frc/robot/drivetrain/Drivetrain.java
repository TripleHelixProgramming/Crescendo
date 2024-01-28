// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  static double kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static double kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          "FrontLeft",
          DriveConstants.MotorControllers.kFrontLeftDriveMotorPort,
          DriveConstants.MotorControllers.kFrontLeftTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kFrontLeftTurningEncoderPort);
  private final SwerveModule m_frontRight =
      new SwerveModule(
          "FrontRight",
          DriveConstants.MotorControllers.kFrontRightDriveMotorPort,
          DriveConstants.MotorControllers.kFrontRightTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kFrontRightTurningEncoderPort);
  private final SwerveModule m_rearLeft =
      new SwerveModule(
          "RearLeft",
          DriveConstants.MotorControllers.kRearLeftDriveMotorPort,
          DriveConstants.MotorControllers.kRearLeftTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kRearLeftTurningEncoderPort);
  private final SwerveModule m_rearRight =
      new SwerveModule(
          "RearRight",
          DriveConstants.MotorControllers.kRearRightDriveMotorPort,
          DriveConstants.MotorControllers.kRearRightTurningMotorPort,
          DriveConstants.AbsoluteEncoders.kRearRightTurningEncoderPort);

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

  private final AHRS m_gyro = new AHRS();

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();

    for (SwerveModule module : modules) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();
    }
  }

  @Override
  public void periodic() {
    updateOdometry();

    for (SwerveModule module : modules) {
      SmartDashboard.putNumber(
          module.getName() + "RelativeTurningPosition",
          module.getRelativeTurningPosition().getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "AbsoluteTurningPosition",
          module.getAbsTurningPosition(0.0).getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "RelativeDrivePosition", module.getRelativeDrivePosition());

      SmartDashboard.putNumber(
          module.getName() + "AbsoluteMagnetOffset",
          module.getAbsTurningEncoderOffset().getDegrees());
    }

    SmartDashboard.putNumber("GyroAngle", m_gyro.getRotation2d().getDegrees());
  }

  /**
   * @param chassisSpeeds Robot-relative chassis speeds (x, y, theta)
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /** Reconfigures all swerve module steering angles using external alignment device */
  public void zeroAbsTurningEncoderOffsets() {
    for (SwerveModule module : modules) {
      module.zeroAbsTurningEncoderOffset();
    }
  }

  /**
   * @return The direction of the robot pose
   */
  public Rotation2d getHeading() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * @return The robot pose
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * @param pose The robot pose
   */
  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  /**
   * @param alliance The current alliance color
   */
  public void setHeading(Alliance alliance) {
    Pose2d pose;

    switch (alliance) {
      case RED_ALLIANCE:
        pose = new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI));
      case BLUE_ALLIANCE:
      default:
        pose = new Pose2d(getPose().getTranslation(), new Rotation2d());
    }

    m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(), pose);
  }

  /**
   * @return Array of swerve module positions
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] bill = {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
    return bill;
  }

  /**
   * @return Array of swerve module states
   */
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
    }

    return states;
  }

  /**
   * @return The current robot-relative chassis speeds (x, y, theta)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::drive,

        // update these to fit our robot
        new HolonomicPathFollowerConfig(
            // Translation PID constants
            new PIDConstants(
                Constants.ModuleConstants.kDriveP,
                Constants.ModuleConstants.kDriveI,
                Constants.ModuleConstants.kDriveD),
            // Rotation PID constants
            new PIDConstants(
                Constants.ModuleConstants.kTurningP,
                Constants.ModuleConstants.kDriveI,
                Constants.ModuleConstants.kDriveD),
            Constants.DriveConstants.kMaxTranslationalVelocity, // Max module speed, in m/s
            Constants.DriveConstants.kRadius, // Drive base radius in meters
            new ReplanningConfig() // Default path replanning config
            ),
        () -> {
          return false;
        }, // Never mirror path
        this // Reference to this subsystem to set requirements
        );
  }
}
