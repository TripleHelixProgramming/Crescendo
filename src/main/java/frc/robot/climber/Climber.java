package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final ClimberSide m_leftClimberSide = new ClimberSide(
        ClimberConstants.kLeftClimberMotorPort);
    private final ClimberSide m_rightClimberSide = new ClimberSide(
        ClimberConstants.kRightClimberMotorPort);

    private ClimberSide[] climberSides = {m_leftClimberSide, m_rightClimberSide};

    // DifferentialDrive m_arcadeDrive;

    public Climber() {


 
        //m_arcadeDrive = new DifferentialDrive(m_leftClimberSide, m_rightClimberSide);
    }

    public void configureUpperLimit(boolean upperLImitEnabled) {
        for (ClimberSide climberSide : climberSides) {
            climberSide.configureUpperLimit(upperLImitEnabled);
          }
    }

    // private void arcadeDrive(XboxController xboxController) {
    //     m_arcadeDrive.arcadeDrive(-xboxController.getRightY(), -xboxController.getLeftX());
    // }

    // public CANSparkMax[] getMotors() {
    //     CANSparkMax[] motors = {m_leftClimberMover, m_rightClimberMover};
    //     return motors;
    // }

    // public SparkPIDController[] getPIDControllers() {
    //     SparkPIDController[] PIDControllers = {m_leftClimberPIDController, m_rightClimberPIDController};
    //     return PIDControllers;
    // }

    // public Command createArcadeDriveCommand(XboxController xboxController) {
    //     return this.run(() -> this.arcadeDrive(xboxController));
    // }
}