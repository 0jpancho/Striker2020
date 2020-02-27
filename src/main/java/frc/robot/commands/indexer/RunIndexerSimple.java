package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class RunIndexerSimple extends CommandBase {

    private Indexer m_indexer;
    private double power;

    public RunIndexerSimple(Indexer indexer, double power) {
        this.m_indexer = indexer;
        this.power = power;

        addRequirements(indexer);
    }

    public void initialize() {
        m_indexer.setMotors(ControlMode.PercentOutput, 0);
    }

    public void execute() {
        m_indexer.setMotors(ControlMode.PercentOutput, power);
    }

    public void end(boolean interrupted) {
        m_indexer.setMotors(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished() {
        return false;
    }
}