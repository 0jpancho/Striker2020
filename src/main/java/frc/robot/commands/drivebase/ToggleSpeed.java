package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivebase;

public class ToggleSpeed extends InstantCommand {

    Drivebase m_drive = new Drivebase();

    public ToggleSpeed(Drivebase drive) {
        this.m_drive = drive;

        addRequirements(drive);
    }

}