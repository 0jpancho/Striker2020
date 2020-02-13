package frc.robot.commands.drivebase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class DriveByDistance extends CommandBase{
    
    DriveBase drive = new DriveBase();

    
    private double targetDistance; //meters
    private double tolerance = 100; //ticks

    public DriveByDistance(DriveBase drive, double targetDistance){
        this.drive = drive;
        this.targetDistance = targetDistance;
    }

    @Override
    public void initialize() {
        drive.setDriveConfig(ControlMode.PercentOutput, 0);
        targetDistance *= Constants.DriveConstants.kMetersPerCount;
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.setDriveConfig(ControlMode.Position, targetDistance);
    }
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if ((drive.getLPosTicks() - tolerance < drive.getLPosTicks() && drive.getLPosTicks() < drive.getLPosTicks() + tolerance) &&
             drive.getRPosTicks() - tolerance < drive.getRPosTicks() && drive.getRPosTicks() < drive.getRPosTicks() + tolerance){

            return true;
        }

        else{
            return false;
        }
    }
}