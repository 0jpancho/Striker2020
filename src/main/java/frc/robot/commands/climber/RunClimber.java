package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {

    private Climber m_climber;
    private XboxController m_controller;

    public RunClimber(Climber climber, XboxController controller) {
        m_climber = climber;
        m_controller = controller;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setWinchPower(0);
    }

    @Override
    public void execute() {
        
        if (m_controller.getYButton()) {
            m_climber.setLiftPower(Constants.Winch.kLiftPower);
        }

        else if (m_controller.getAButton()) {
            m_climber.setLiftPower(-0.25);
        }

        else {
            m_climber.setWinchPower(0);
        }

        if (m_controller.getStickButton(Hand.kLeft)) {
            
            m_climber.setWinchPower(-Constants.Winch.kWinchPower);
        }

        else if (m_controller.getStickButton(Hand.kRight)) {
            m_climber.setWinchPower(-0.5);
        }

        else {
            m_climber.setLiftPower(0);
        }

       
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setWinchPower(0);
        m_climber.setLiftPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}