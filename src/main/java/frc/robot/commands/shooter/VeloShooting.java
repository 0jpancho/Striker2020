package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class VeloShooting extends CommandBase {

    private Shooter m_shooter;
    private double kInputRPM;
    private double targetCountsPer100ms;

    public VeloShooting(Shooter shooter, double inputRPM) {
        m_shooter = shooter;
        kInputRPM = inputRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Scale RPM input with units per 100ms (hence / 600)
        targetCountsPer100ms = (kInputRPM * Constants.Shooter.kEncoderResolution) / 600;
        System.out.println("Target 'RPM': " + targetCountsPer100ms);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //m_shooter.setMotors(ControlMode.Velocity, targetCountsPer100ms);
        m_shooter.getLeftMotor().set(ControlMode.Velocity, targetCountsPer100ms);
        m_shooter.getRightMotor().set(ControlMode.Velocity, targetCountsPer100ms);
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