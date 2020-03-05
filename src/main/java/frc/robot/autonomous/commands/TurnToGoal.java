package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.vision.Limelight;

public class TurnToGoal extends CommandBase {

    private Drivebase m_drive;
    private Limelight m_limelight;
    private double kP = .1f;
    private double minPower = 0.05f;

    private double leftPower;
    private double rightPower;

    private double headingError;

    public TurnToGoal(Drivebase drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        addRequirements(m_drive, m_limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        headingError = m_limelight.getDegRotationToTarget();
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
        return headingError < 2;
    }
}