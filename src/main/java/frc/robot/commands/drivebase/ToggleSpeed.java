package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class ToggleSpeed extends CommandBase {

    private Drivebase m_drive = new Drivebase();
    private boolean isFast = true;

    public ToggleSpeed(Drivebase drive) {
        m_drive = drive;

        addRequirements(drive);
    }

    public void initialize() {

        if (isFast) {
            m_drive.getLeftMaster().configPeakOutputForward(1);
            m_drive.getRightMaster().configPeakOutputReverse(-1);
        }

        else {
            m_drive.getLeftMaster().configPeakOutputForward(1);
            m_drive.getRightMaster().configPeakOutputReverse(-1);
        }
    }

    public void end() {
        
    }

    public boolean isFinised() {
        return false;
    }

}