package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class DriveByDistance extends CommandBase {

    private Drivebase m_drive = new Drivebase();

    private double targetMeters; // meters
    private double tolerance = 100; // ticks

    public DriveByDistance(Drivebase drive, double targetMeters) {
        m_drive = drive;
        this.targetMeters = targetMeters;

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
        
        targetMeters *= Constants.Drive.kMetersPerCount;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.configMotors(ControlMode.Position, targetMeters);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.configMotors(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(m_drive.getLPosTicks() - targetMeters) < tolerance
                && Math.abs(m_drive.getRPosTicks() - targetMeters) < tolerance) {
            return true;
        } else {
            return false;
        }
    }
}