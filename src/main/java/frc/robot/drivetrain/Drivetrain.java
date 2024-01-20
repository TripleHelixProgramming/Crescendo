// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
// auto impors
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  static double kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static double kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;

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
    resetGyro();

    for (SwerveModule module : modules) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();
    }

  }

  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
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
   * @param chassisSpeeds x, y, and theta speeds
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
   * @return The heading of the robot
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose) {
    resetGyro();
    m_odometry.resetPosition(getHeading(),getSwerveModulePositions(),pose);
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
    SwerveModulePosition[] bill = {   
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()};
    return bill;
  }


  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
    }

    return states;
  }
  
  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return toChassisSpeeds(getHeading(), this.vxMetersPerSecond, this.vyMetersPerSecond);
  }
  

  public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            ()->this.getPose(), // Robot pose supplier
            this::resetOdometry,
             this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
  }

}
