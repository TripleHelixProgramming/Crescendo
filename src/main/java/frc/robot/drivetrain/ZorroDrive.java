package frc.robot.drivetrain;

// import frc.robot.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;

// import com.team2363.utilities.ControllerMap;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZorroDrive extends Drive {

  Joystick m_controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private boolean fieldRelative;

  public ZorroDrive(Drivetrain subsystem, Joystick joysticks, boolean fieldRelative) {
    super(subsystem);
    this.m_controller = joysticks;
    this.fieldRelative = fieldRelative;
  }

  @Override
  public double getX() {
    return -m_xspeedLimiter.calculate(
        MathUtil.applyDeadband(-m_controller.getRawAxis(OIConstants.kZorroRightYAxis), 0.02));
  }

  @Override
  public double getY() {
    return -m_yspeedLimiter.calculate(
        MathUtil.applyDeadband(-m_controller.getRawAxis(OIConstants.kZorroLeftXAxis), 0.02));
  }

  @Override
  public double getTheta() {
    return -m_rotLimiter.calculate(
        MathUtil.applyDeadband(-m_controller.getRawAxis(OIConstants.kZorroLeftXAxis), 0.02));
  }

  @Override
  public boolean getFieldRelative() {
    return fieldRelative;
  }
}
