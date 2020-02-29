package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class RawArcadeDrive extends CommandBase {

    private final Drivebase m_drive;
    private final XboxController m_controller;

    private SlewRateLimiter inputs = new SlewRateLimiter(0.5);

    public RawArcadeDrive(Drivebase drive, XboxController controller) {
        m_drive = drive;
        m_controller = controller;

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
        m_drive.rawArcadeDrive(-m_controller.getY(Hand.kLeft) * 0.75, m_controller.getX(Hand.kRight) * 0.75);
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