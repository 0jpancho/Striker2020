package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class SimpleFeed extends CommandBase {
    
    private Indexer indexer = new Indexer();
    private double power;

    public SimpleFeed(Indexer indexer, double power){
        this.indexer = indexer;
        this.power = power;

        addRequirements(indexer);
    }

    public void initialize(){
        indexer.configMotors(ControlMode.PercentOutput, 0);
    }

    public void execute(){
        indexer.configMotors(ControlMode.PercentOutput, power);
    }

    public void end(boolean interrupted){
        indexer.configMotors(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished(){
        return false;
    }
}