package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class VeloShooting extends CommandBase{
    
    private Shooter shooter = new Shooter();
    private double inputRPM;
    private double targetRPM;

    public VeloShooting(Shooter shooter, double inputRPM){
        this.shooter = shooter;
        this.inputRPM = inputRPM;
    }
    
    @Override
    public void initialize() {
        //Scale RPM input with units per 100ms (hence / 600)
        targetRPM = (inputRPM * Constants.ShooterConstants.kEncoderResolution) / 600;
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.configMotors(ControlMode.Velocity, targetRPM);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.configMotors(ControlMode.PercentOutput, 0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}