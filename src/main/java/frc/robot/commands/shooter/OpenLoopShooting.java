package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class OpenLoopShooting extends CommandBase {

    private Shooter m_shooter;
    private double power;

    public OpenLoopShooting(Shooter shooter, double power) {
        this.m_shooter = shooter;
        this.power = power;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setMotors(ControlMode.PercentOutput, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setMotors(ControlMode.PercentOutput, power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setMotors(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}