package frc.robot.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class OpenLoopShooting extends CommandBase{

    private Shooter shooter = new Shooter();
    private double power;

    public OpenLoopShooting(Shooter shooter, double power){
       this.shooter = shooter;
       this.power = power;

       addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.configMotors(ControlMode.PercentOutput, 0);
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.configMotors(ControlMode.PercentOutput, power);
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