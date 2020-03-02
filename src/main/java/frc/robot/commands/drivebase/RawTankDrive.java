package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class RawTankDrive extends CommandBase {

    private final Drivebase m_drive;
    private final XboxController m_controller;

    private final SlewRateLimiter m_leftLimiter = new SlewRateLimiter(0.5);
    private final SlewRateLimiter m_rightLimiter = new SlewRateLimiter(0.5);

    public RawTankDrive(Drivebase drive, XboxController controller) {
        m_drive = drive;
        m_controller = controller;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.resetHeading();
        m_drive.resetOdometry();
    }

    @Override
    public void execute() {
        DoubleSupplier left = () -> m_leftLimiter.calculate(m_controller.getY(Hand.kLeft));
        DoubleSupplier right = () -> m_rightLimiter.calculate(m_controller.getY(Hand.kRight));

        if (Math.abs(left.getAsDouble()) < 0.05) {
            left = () -> 0;
          }
      
          if (Math.abs(right.getAsDouble()) < 0.05) {
            right = () -> 0;
          }

        m_drive.getLeftMaster().set(ControlMode.PercentOutput, left.getAsDouble());
		m_drive.getRightMaster().set(ControlMode.PercentOutput, right.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}