package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

  private final Drivetrain m_swerve = new Drivetrain();

  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public RobotContainer() {

    // m_swerve.setDefaultCommand(
    //     // A split-stick arcade command, with forward/backward controlled by the left
    //     // hand, and turning controlled by the right.
    //     new RunCommand(
    //         () ->
    //             m_swerve.arcadeDrive(
    //                 -m_driverController.getLeftY(), -m_driverController.getRightX()),
    //         m_swerve));

    // Create a button on Smart Dashboard to reset the encoders.
    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> m_swerve.setAbsTurningEncoderZero()).ignoringDisable(true));
  }

  // private void driveWithJoystick(boolean fieldRelative) {
  //   // Get the x speed. We are inverting this because Xbox controllers return
  //   // negative values when we push forward.
  //   final var xSpeed =
  //       -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
  //           * Drivetrain.kMaxSpeed;

  //   // Get the y speed or sideways/strafe speed. We are inverting this because
  //   // we want a positive value when we pull to the left. Xbox controllers
  //   // return positive values when you pull to the right by default.
  //   final var ySpeed =
  //       -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
  //           * Drivetrain.kMaxSpeed;

  //   // Get the rate of angular rotation. We are inverting this because we want a
  //   // positive value when we pull to the left (remember, CCW is positive in
  //   // mathematics). Xbox controllers return positive values when you pull to
  //   // the right by default.
  //   final var rot =
  //       -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
  //           * Drivetrain.kMaxAngularSpeed;

  //   m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  // }

  public Command getAutonomousCommand() {
    Command autoCommand = null;
    return autoCommand;
  }
}
