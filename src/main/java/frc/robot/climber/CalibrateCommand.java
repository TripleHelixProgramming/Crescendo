// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class CalibrateCommand extends Command {

  // The subsystem the command runs on
  private final Climber m_climber;
  
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

    // if (motor.getUpperLimitDetected & !m_leftMotorFinished)
    
    // m_climber.getLeftMotor().getUpperLimitDetected() & 



    // for each ClimberSide,
          // if the ClimberSide has finished calibrating
                // do nothing
          // else
                // if upperLimitDetected
                    // resetEncoder
                    // set voltage 0
                    // set FinishedCalibrating true
                // else
                    // set voltage 2
          

  }

  @Override
  public boolean isFinished() {
    // return (leftclimberside has finished calibrating) & (rightclimberside has finished calibrating)
  }

  @Override
  public void end(boolean interrupeted) {
    m_climber.configureUpperLimit(true);
  }
}
