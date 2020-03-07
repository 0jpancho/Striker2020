package frc.robot.autonomous.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RevThenShoot extends CommandBase {

    private Indexer m_indexer;
    private Shooter m_shooter;


    double targetCountsPer100ms = (Constants.Shooter.kRPM * Constants.Shooter.kEncoderResolution) / 600;

    public RevThenShoot(Indexer indexer, Shooter shooter) {
        m_indexer = indexer;
        m_shooter = shooter;

        addRequirements(indexer, shooter);
    }

    public void initialize() {
        // Scale RPM input with units per 100ms (hence / 600)
        
    }
  
    public void execute() {
        m_shooter.getLeftMotor().set(ControlMode.Velocity, targetCountsPer100ms);
        m_shooter.getRightMotor().set(ControlMode.Velocity, targetCountsPer100ms);


        /*
        if (m_shooter.getLeftMotor().getClosedLoopError() < 300 && m_shooter.getRightMotor().getClosedLoopError() < 300) {
            m_indexer.getMaster().set(ControlMode.PercentOutput, Constants.Indexer.kPower);
            m_indexer.getSlave().set(ControlMode.PercentOutput, Constants.Indexer.kPower);
        }
        */
    }
  
  
    public void end(boolean interrupted) {
        //m_indexer.stop();
        m_shooter.stop();
    }

    public boolean isFinished() {
      return false;
    }
    
}