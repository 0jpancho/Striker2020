package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class RunDrive extends CommandBase {
    
    private Drivebase m_drive;
    private double kPower;
    
    public RunDrive (Drivebase drive, double power) {
        m_drive = drive;
        kPower = power;
    }

    public void initialize() {

    }

    public void execute() {

        m_drive.getLeftMaster().set(kPower);
        m_drive.getRightMaster().set(kPower);

    }

    public void end() {

    }

    public boolean isFinished() {
        return false;
    }
}