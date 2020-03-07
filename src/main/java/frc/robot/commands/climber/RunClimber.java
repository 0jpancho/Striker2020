package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {

    private Climber m_climber;
    private XboxController m_controller;

    DoubleSupplier downPow = () -> -1;

    DoubleSupplier upPow = () -> 1;
  
    DoubleSupplier stop = () -> 0;

    public RunClimber(Climber climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
                
        if (m_controller.getYButton()) {
            m_climber.setLiftPower(upPow);
        }

        else if (m_controller.getAButton()) {
            m_climber.setLiftPower(downPow);
        }
        
        else {
            m_climber.setLiftPower(stop);
        }
        
        if (m_controller.getStickButton(Hand.kLeft)) {
            
            m_climber.setWinchPower(1);
        }

        else if (m_controller.getStickButton(Hand.kRight)) {
            m_climber.setWinchPower(-0.5);
        }

        else {
            m_climber.setWinchPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setWinchPower(0);
        //m_climber.setLiftPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}