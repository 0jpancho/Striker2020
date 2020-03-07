package frc.robot.autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.commands.RunDrive;
import frc.robot.subsystems.Drivebase;

public class DriveByTime extends SequentialCommandGroup {
    
    public DriveByTime(Drivebase drive, double power, double timeout) {

        addCommands(
        
        );
    }
}