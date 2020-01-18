package frc.robot.commands.teleop;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;

public class SimpleArcade extends CommandBase {

    private final DriveTrain m_driveTrain;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_rotation;

    public SimpleArcade(DriveTrain driveTrain, DoubleSupplier forward, DoubleSupplier rotation){
        m_driveTrain = driveTrain;
        m_forward = forward;
        m_rotation = rotation;
        addRequirements(driveTrain);
    }

     // Called when the command is initially scheduled.
    @Override
    public void initialize() {


    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.ArcadeDrive(m_forward, m_rotation);
    }
   
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}