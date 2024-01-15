package frc.robot.drivetrain;

// import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;

// import com.team2363.utilities.ControllerMap;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZorroDrive extends Drive {

  Joystick m_controller;

  private boolean fieldRelative;

  public ZorroDrive(Drivetrain subsystem, Joystick joysticks, boolean fieldRelative) {
    super(subsystem);
    this.m_controller = joysticks;
    this.fieldRelative = fieldRelative;
  }

  @Override
  public double getX() {
    return -m_controller.getRawAxis(OIConstants.kZorroRightYAxis);
  }

  @Override
  public double getY() {
    return -m_controller.getRawAxis(OIConstants.kZorroLeftXAxis);
  }

  @Override
  public double getTheta() {
    return -m_controller.getRawAxis(OIConstants.kZorroLeftXAxis);
  }

  @Override
  public boolean getFieldRelative() {
    return fieldRelative;
  }
}
