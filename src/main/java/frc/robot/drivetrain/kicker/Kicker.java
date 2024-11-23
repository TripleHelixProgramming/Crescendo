package frc.robot.drivetrain.kicker;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants.KickerConstants;

public class Kicker {

    private final Solenoid m_kicker;

    public Kicker() {

        m_kicker = new Solenoid(PneumaticsModuleType.CTREPCM, KickerConstants.kKickerDeployPort);

    }

    private void deploy() {
        m_kicker.set(true);
    }

    private void retract() {
        m_kicker.set(false);
    }

    public Command createDeployCommand() {
        return new InstantCommand(() -> deploy());
    }

    public Command createRetractCOmmand() {
        return new InstantCommand(() -> retract());
    }
}
