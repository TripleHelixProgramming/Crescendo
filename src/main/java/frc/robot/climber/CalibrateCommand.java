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
    for (ClimberSide climberSide : m_climber.getClimberSides()) {
      climberSide.configureUpperLimit(false);
      climberSide.setHasFinishedCalibrating(false);
    }
  }

  @Override
  public void execute() {
    for (ClimberSide climberSide : m_climber.getClimberSides()) {
      if (climberSide.getHasFinishedCalibrating()) {

      } else {
        if (climberSide.getCurrentSenseState()) {
          climberSide.resetEncoder();
          climberSide.stop();
          climberSide.setHasFinishedCalibrating(true);
        } else {
          climberSide.driveSlowlyTo(100);
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_climber.bothSidesCalibrated();
  }

  @Override
  public void end(boolean interrupted) {
    for (ClimberSide climberSide : m_climber.getClimberSides()) {
      climberSide.configureUpperLimit(true);
    }
  }
}
