package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;

public class Climber {
    private final CANSparkMax m_leftClimberMover;
    private final CANSparkMax m_rightClimberMover;

    private final RelativeEncoder m_leftClimberRelativeEncoder;
    private final RelativeEncoder m_rightClimberRelativeEncoder;

    private final SparkPIDController m_leftClimberPIDController;
    private final SparkPIDController m_rightClimberPIDController;

    public Climber() {
        m_leftClimberMover = new CANSparkMax(ClimberConstants.kLeftClimberMotorPort, MotorType.kBrushless);
        m_rightClimberMover = new CANSparkMax(ClimberConstants.kRightClimberMotorPort, MotorType.kBrushless);

        m_leftClimberMover.restoreFactoryDefaults();
        m_rightClimberMover.restoreFactoryDefaults();

        m_leftClimberMover.setIdleMode(IdleMode.kBrake);
        m_rightClimberMover.setIdleMode(IdleMode.kBrake);

        m_leftClimberMover.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
        m_rightClimberMover.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);

        m_leftClimberMover.setInverted(false);
        m_rightClimberMover.setInverted(false);

        m_leftClimberPIDController = m_leftClimberMover.getPIDController();
        m_rightClimberPIDController = m_rightClimberMover.getPIDController();

        m_leftClimberPIDController.setP(ClimberConstants.kClimberP);
        m_leftClimberPIDController.setI(ClimberConstants.kClimberI);
        m_leftClimberPIDController.setD(ClimberConstants.kClimberD);

        m_rightClimberPIDController.setP(ClimberConstants.kClimberP);
        m_rightClimberPIDController.setI(ClimberConstants.kClimberI);
        m_rightClimberPIDController.setD(ClimberConstants.kClimberD);

        m_leftClimberRelativeEncoder = m_leftClimberMover.getEncoder();
        m_rightClimberRelativeEncoder = m_rightClimberMover.getEncoder();

        m_leftClimberRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversionFactor);
        m_rightClimberRelativeEncoder.setPositionConversionFactor(ClimberConstants.kClimberPositionConversionFactor);

        m_leftClimberRelativeEncoder.setVelocityConversionFactor(ClimberConstants.kClimberVelocityConversionFactor);
        m_rightClimberRelativeEncoder.setVelocityConversionFactor(ClimberConstants.kClimberVelocityConversionFactor);
    }
}