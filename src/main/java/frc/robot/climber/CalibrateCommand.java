// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkMax;

public class CalibrateCommand extends Command {

  // The subsystem the command runs on
  public final Climber m_climber;

  private boolean m_leftMotorFinished = false;
  private boolean m_rightMotorFinished = false;

  public CalibrateCommand(Climber subsystem) {
    m_climber = subsystem;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.configureUpperLimit(false);
  }

  @Override
  public void execute() {
    // for (CANSparkMax motor : m_climber.getMotors()) {
    //   if (m_climber.getUpperLimitDetected(motor) & !m_leftMotorFinished) {
    //     motor.setVoltage(0.0);
    //   } else {
    //     motor.setVoltage(2.0);
    //   }
    // }
    if (m_climber.getUpperLimitDetected(m_climber.getMotors()))
  }

  @Override
  public void end(boolean interrupeted) {
    m_climber.resetEncoders();
    m_climber.configureUpperLimit(true);
  }
}
