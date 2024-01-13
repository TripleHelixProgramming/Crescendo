package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public abstract class Drive extends Command {

    private double xDot;
    private double yDot;
    private double thetaDot;
    private boolean fieldRelative;
    private ChassisSpeeds chassisSpeeds;

    // The subsystem the command runs on
    public final Drivetrain drivetrain;

    public Drive(Drivetrain subsystem){
        drivetrain = subsystem;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }
            
    @Override
    public void execute() {
        xDot = getX() * DriveConstants.kMaxTranslationalVelocity;
        yDot = getY() * DriveConstants.kMaxTranslationalVelocity;
        thetaDot = getTheta() * DriveConstants.kMaxRotationalVelocity;
        fieldRelative = getFieldRelative();

        chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, drivetrain.getHeading())
            : new ChassisSpeeds(xDot, yDot, thetaDot);
        
        drivetrain.drive(chassisSpeeds, true);
    }

    abstract public double getX();
    abstract public double getY();
    abstract public double getTheta();
    abstract public boolean getFieldRelative();
}
