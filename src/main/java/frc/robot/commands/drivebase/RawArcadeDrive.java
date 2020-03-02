package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class RawArcadeDrive extends CommandBase {

    private final Drivebase m_drive;
    private final XboxController m_controller;

    private final SlewRateLimiter m_forwardLimiter = new SlewRateLimiter(0.5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0.5);

    public RawArcadeDrive(Drivebase drive, XboxController controller) {
        m_drive = drive;
        m_controller = controller;

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.resetHeading();
        m_drive.resetOdometry();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        DoubleSupplier forward = () -> m_forwardLimiter.calculate(m_controller.getY(Hand.kLeft));
        DoubleSupplier rot = () -> m_rotLimiter.calculate(m_controller.getX(Hand.kRight));

        if (Math.abs(forward.getAsDouble()) < 0.05) {
            forward = () -> 0;
          }
      
          if (Math.abs(rot.getAsDouble()) < 0.05) {
            rot = () -> 0;
          }

        m_drive.getLeftMaster().set(ControlMode.PercentOutput, forward.getAsDouble() + rot.getAsDouble());
		m_drive.getRightMaster().set(ControlMode.PercentOutput, forward.getAsDouble() - rot.getAsDouble());
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