// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.CalibrationState;
import frc.robot.climber.Climber;
import frc.robot.climber.ClimberSide;

public class CalibrateCommand extends Command {

  private final Climber m_climber;
  private final ClimberSide[] m_actuators;

  public CalibrateCommand(Climber subsystem) {
    m_climber = subsystem;
    addRequirements(m_climber);
    m_actuators = m_climber.getClimberSides();
  }

  @Override
  public void initialize() {
    for (ClimberSide actuator : m_actuators) {
      actuator.configureUpperLimit(false);
      actuator.configurePositionController(
          ClimberConstants.slowConstraints, ClimberConstants.kSeekPosition);
      actuator.setCalibrationState(CalibrationState.UNCALIBRATED);
    }
  }

  @Override
  public void execute() {
    for (ClimberSide actuator : m_actuators) {
      if (actuator.getCalibrationState() != CalibrationState.CALIBRATED) calibrate(actuator);
    }
  }

  private void calibrate(ClimberSide actuator) {
    if (actuator.getCurrentSenseState()) {
      actuator.stop();
      actuator.resetEncoder();
      actuator.setCalibrationState(CalibrationState.CALIBRATED);
    } else {
      actuator.driveToTargetPosition();
      actuator.setCalibrationState(CalibrationState.HOMING);
    }
  }

  @Override
  public boolean isFinished() {
    for (ClimberSide actuator : m_actuators)
      if (actuator.getCalibrationState() != CalibrationState.CALIBRATED) return false;
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    for (ClimberSide actuator : m_actuators) actuator.configureUpperLimit(true);
  }
}
