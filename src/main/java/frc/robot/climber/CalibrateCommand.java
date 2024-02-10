// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;

public class CalibrateCommand extends Command {

  // The subsystem the command runs on
  private final Climber m_climber;

  public CalibrateCommand(Climber subsystem) {
    m_climber = subsystem;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    for (ClimberSide climberSide : m_climber.getClimberSides()) {
      climberSide.configureUpperLimit(false);
      climberSide.setHasFinishedCalibrating(false);
    }
  }

  @Override
  public void execute() {
    for (ClimberSide climberSide : m_climber.getClimberSides()) {
      if (climberSide.getHasFinishedCalibrating()) {
        return;
      } else {
        if (climberSide.getUpperHardStopDetected()) {
          climberSide.resetEncoder();
          climberSide.stop();
          climberSide.setHasFinishedCalibrating(true);
        } else {
          climberSide.setVoltage(ClimberConstants.kCalibrationVoltage);
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_climber.getClimberSides()[0].getHasFinishedCalibrating()
        && m_climber.getClimberSides()[1].getHasFinishedCalibrating();
  }

  @Override
  public void end(boolean interrupeted) {
    for (ClimberSide climberSide : m_climber.getClimberSides()) {
      climberSide.configureUpperLimit(true);
    }
  }
}
