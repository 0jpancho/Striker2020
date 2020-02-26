package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.vision.Limelight;

public class TurnToGoal extends CommandBase {

    private DriveBase m_drive = new DriveBase();
    private Limelight m_limelight = new Limelight();
    private double kP = .1f;
    private double minPower = 0.05f;

    double leftPower;
    double rightPower;

    public TurnToGoal(DriveBase m_drive, Limelight m_limelight) {
        this.m_drive = m_drive;
        this.m_limelight = m_limelight;

        addRequirements(m_drive, m_limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        double x = m_limelight.getDegRotationToTarget();

        double headingError = x;
        double turnCorrect = 0.0f;

        if (headingError > 1.0) {
            turnCorrect = kP * headingError - minPower;
        }

        else if (headingError < 1.0) {
            turnCorrect = kP * headingError + minPower;
        }

        leftPower += turnCorrect;
        rightPower -= turnCorrect;

        m_drive.getLeftMaster().set(ControlMode.PercentOutput, leftPower);
        m_drive.getRightMaster().set(ControlMode.PercentOutput, rightPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.configMotors(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}