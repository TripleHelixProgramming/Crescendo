package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberSide m_leftClimberSide = new ClimberSide(
        ClimberConstants.kLeftClimberMotorPort);
    private final ClimberSide m_rightClimberSide = new ClimberSide(
        ClimberConstants.kRightClimberMotorPort);

    private ClimberSide[] climberSides = {m_leftClimberSide, m_rightClimberSide};

    DifferentialDrive m_arcadeDrive;

    public Climber() {



        //m_arcadeDrive = new DifferentialDrive(m_leftClimberSide, m_rightClimberSide);
    }

    public void stop() {
        for (ClimberSide climberSide : climberSides) {
            climberSide.stop();
        }
    }

    public ClimberSide[] getClimberSides() {
        return climberSides;
    }

    public Command createStopCommand() {
        return this.runOnce(() -> this.stop());
    }


    // public Command createSetVelocityCommand(XboxController xboxController) {
    //     return this.run(() -> this.setVelocity(xboxController.getRightY()));
    // }

    // public Command createSetVoltageCommand(XboxController xboxController) {
    //     return this.run(() -> this.setVoltage(xboxController.getRightY()));
    // }

    public Command createSetPositionCommand(double targetPosition) {
        return this.runOnce(() -> {
            m_leftClimberSide.setPosition(targetPosition);
            m_rightClimberSide.setPosition(targetPosition);
        });
    }

    private void arcadeDrive(XboxController xboxController) {
        m_arcadeDrive.arcadeDrive(-xboxController.getRightY(), -xboxController.getLeftX());
    }

    // public CANSparkMax[] getMotors() {
    //     CANSparkMax[] motors = {m_leftClimberMover, m_rightClimberMover};
    //     return motors;
    // }

    // public SparkPIDController[] getPIDControllers() {
    //     SparkPIDController[] PIDControllers = {m_leftClimberPIDController, m_rightClimberPIDController};
    //     return PIDControllers;
    // }

    public Command createArcadeDriveCommand(XboxController xboxController) {
        return this.run(() -> this.arcadeDrive(xboxController));
    }
}