package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class VeloShooting extends CommandBase {

    private Shooter m_shooter = new Shooter();
    private double inputRPM;
    private double targetCountsPer100ms;

    public VeloShooting(Shooter shooter, double inputRPM) {
        this.m_shooter = shooter;
        this.inputRPM = inputRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Scale RPM input with units per 100ms (hence / 600)
        targetCountsPer100ms = (inputRPM * Constants.Shooter.kEncoderResolution) / 600;
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.setMotors(ControlMode.Velocity, targetCountsPer100ms);
        //SmartDashboard.putNumber("L Velocity", m_shooter.)
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