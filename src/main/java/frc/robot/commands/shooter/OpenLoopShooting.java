package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class OpenLoopShooting extends CommandBase {

    private Shooter m_shooter;
    private double kPower;

    public OpenLoopShooting(Shooter shooter, double power) {
        m_shooter = shooter;
        kPower = power;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.getLeftMotor().set(kPower);
        m_shooter.getRightMotor().set(kPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}