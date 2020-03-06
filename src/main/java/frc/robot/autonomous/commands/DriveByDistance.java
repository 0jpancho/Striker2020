package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class DriveByDistance extends CommandBase {

    private Drivebase m_drive = new Drivebase();

    private double m_targetMeters; // meters
    private double tolerance = 100; // ticks

    

    public DriveByDistance(Drivebase drive, double targetMeters) {
        System.out.println("CONSTRUCTOR");
        m_drive = drive;
        m_targetMeters = targetMeters;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        m_drive.getLeftMaster().config_kP(0, Constants.Drive.kPosGains.kP);
        m_drive.getLeftMaster().config_kI(0, Constants.Drive.kPosGains.kI);
        m_drive.getLeftMaster().config_kD(0, Constants.Drive.kPosGains.kD);
        m_drive.getLeftMaster().config_kF(0, Constants.Drive.kPosGains.kF);

        m_drive.getRightMaster().config_kP(0, Constants.Drive.kPosGains.kP);
        m_drive.getRightMaster().config_kI(0, Constants.Drive.kPosGains.kI);
        m_drive.getRightMaster().config_kD(0, Constants.Drive.kPosGains.kD);
        m_drive.getRightMaster().config_kF(0, Constants.Drive.kPosGains.kF);

        m_drive.stop();
        m_drive.resetOdometry();

        m_targetMeters /= Constants.Drive.kMetersPerCount;
        System.out.println(m_targetMeters);
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("DRIVING");
        m_drive.getLeftMaster().set(ControlMode.Position, m_targetMeters);
        m_drive.getRightMaster().set(ControlMode.Position, m_targetMeters);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_drive.getLeftMaster().getClosedLoopTarget() < 200 && m_drive.getRightMaster().getClosedLoopTarget() < 200) {
             System.out.println("DONE");
           return true;
        } else {
            return false;
        }
    }
}