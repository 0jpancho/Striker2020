package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
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
        m_climber.setClimberPower(0);
    }

    @Override
    public void execute() {
        if (m_controller.getYButton()) {
           m_climber.setClimberPower(Constants.Climber.kClimberPower);
        }

        else {
            m_climber.setClimberPower(0);
        }

        if (m_controller.getXButton()) {
            m_climber.setLiftPower(Constants.Climber.kLiftPower);
        }

        else {
            m_climber.setLiftPower(0);
        }

       
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setClimberPower(0);
        m_climber.setLiftPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}