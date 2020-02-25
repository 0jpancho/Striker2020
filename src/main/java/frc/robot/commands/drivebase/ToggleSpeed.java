package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveBase;

public class ToggleSpeed extends InstantCommand {

    DriveBase m_drive = new DriveBase();

    public ToggleSpeed(DriveBase drive) {
        this.m_drive = drive;

        addRequirements(drive);
    }

}