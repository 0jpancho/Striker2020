package frc.robot.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class RunByTime extends CommandBase {

    private Indexer m_indexer;

    public RunByTime (Indexer indexer) {
        m_indexer = indexer;

        addRequirements(indexer);
    }

    public void initialize() {
        m_indexer.setMotors(ControlMode.PercentOutput, Constants.Indexer.kPower);
    }

    public void execute() {
       
    }

    public void end(boolean interrupted) {
        m_indexer.setMotors(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished() {
        return false;
    }
}