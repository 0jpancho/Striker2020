package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class TurnToGoal extends CommandBase {

    private DriveBase m_drive = new DriveBase();
    private double kP = .1f;
    private double minPower = 0.05f;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    double leftPower;
    double rightPower;

    public TurnToGoal(DriveBase m_drive) {
        this.m_drive = m_drive;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = tx.getDouble(0.0);

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