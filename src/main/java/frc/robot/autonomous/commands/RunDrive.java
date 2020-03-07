package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class RunDrive extends CommandBase {
    
    private Drivebase m_drive;
    private Timer timer = new Timer();
    private double kPower, kDuration;
    
    public RunDrive (Drivebase drive, double power, double duration) {
        m_drive = drive;
        kPower = power;
        kDuration = duration;
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }

    public void execute() {

        if (timer.get() < kDuration) {
            m_drive.getLeftMaster().set(ControlMode.PercentOutput, kPower);
            m_drive.getRightMaster().set(ControlMode.PercentOutput, kPower); 
        }

        else {
            m_drive.stop();
        }
    }

    public void end() {
        m_drive.stop();

    }

    public boolean isFinished() {
        return timer.get() >= kDuration;
    }
}