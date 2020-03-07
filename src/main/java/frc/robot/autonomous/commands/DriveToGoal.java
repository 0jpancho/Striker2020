package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivebase;

public class DriveToGoal extends CommandBase {

    private Drivebase m_drive;
    private Shooter m_shooter;
    private Limelight m_limelight;

    private double targetDist;
    private double kPDist = 0.1;

    public DriveToGoal(Drivebase drive, Shooter shooter, Limelight limelight, double targetDist) {
        m_drive = drive;
        m_limelight = limelight;
        this.targetDist = targetDist;

        addRequirements(drive, limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double distError = targetDist - m_shooter.getDistToTarget(m_limelight.getDegVerticalToTarget());
        double driveCorrect = kPDist * distError;

        double leftPow =+ driveCorrect;
        double rightPow =+ driveCorrect;

        m_drive.getLeftMaster().set(ControlMode.PercentOutput, leftPow);
        m_drive.getRightMaster().set(ControlMode.PercentOutput, rightPow);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}