// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.climber.Climber;
import frc.robot.climber.ClimberSide;

public class DriveToPositionCommand extends Command {

  private final Climber m_climber;
  private final ClimberSide[] m_actuators;
  private final double m_targetPosition;

  public DriveToPositionCommand(Climber subsystem, double targetPosition) {
    m_climber = subsystem;
    addRequirements(m_climber);
    m_actuators = m_climber.getClimberSides();
    this.m_targetPosition = targetPosition;
  }

  @Override
  public void initialize() {
    for (ClimberSide actuator : m_actuators) {
      actuator.configurePositionController(ClimberConstants.rapidConstraints, m_targetPosition);
    }
  }

  @Override
  public void execute() {
    for (ClimberSide actuator : m_actuators) {
      actuator.driveToTargetPosition();
    }
  }

  /**
   * @return True when both climber actuators are within allowable error of setpoint of closed-loop
   *     position controller
   */
  @Override
  public boolean isFinished() {
    for (ClimberSide actuator : m_actuators) if (!actuator.atGoal()) return false;
    return true;
  }
}
