package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {

    private Climber m_climber;

    public RunClimber(Climber climber) {
        this.m_climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.setPower(0);
    }

    @Override
    public void execute() {
        m_climber.setPower(Constants.Climber.kPower);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}