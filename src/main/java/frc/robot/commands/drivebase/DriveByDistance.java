package frc.robot.commands.drivebase;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class DriveByDistance extends CommandBase{
    
    private DriveBase m_drive = new DriveBase();
    
    private double targetDistance; //meters
    private double tolerance = 100; //ticks

    public DriveByDistance(DriveBase drive, double targetDistance){
        this.m_drive = drive;
        this.targetDistance = targetDistance;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.configMotors(ControlMode.PercentOutput, 0);
        targetDistance *= Constants.DriveConstants.kMetersPerCount;
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.configMotors(ControlMode.Position, targetDistance);
    }
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.configMotors(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(Math.abs(m_drive.getLPosTicks() - targetDistance) < tolerance && Math.abs(m_drive.getRPosTicks() - targetDistance) < tolerance){
            return true;
        }
        else{
            return false;
        }
    }
}