package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class TankDrive extends CommandBase {

    private final DriveBase m_drive;
    private final DoubleSupplier left, right;

    public TankDrive(DriveBase drive, DoubleSupplier left, DoubleSupplier right) {
        this.m_drive = drive;
        this.left = left;
        this.right = right;

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_drive.tankDrive(left, right);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}