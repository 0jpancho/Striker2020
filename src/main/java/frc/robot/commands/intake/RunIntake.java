package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {

    private final Intake m_intake;

    public RunIntake(Intake intake) {
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setPower(0);
    }

    @Override
    public void execute() {
        m_intake.setPower(Constants.Intake.kPower);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}