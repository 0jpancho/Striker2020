package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class RawArcadeDrive extends CommandBase {

    private final Drivebase m_drive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public RawArcadeDrive(Drivebase drive, DoubleSupplier forward, DoubleSupplier rotation) {
        m_drive = drive;
        m_forward = forward;
        m_rotation = rotation;

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.resetEncoders();
        m_drive.resetHeading();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.rawArcadeDrive(m_forward, m_rotation);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}