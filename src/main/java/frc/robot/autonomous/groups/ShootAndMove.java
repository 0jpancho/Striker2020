package frc.robot.autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.autonomous.commands.DriveByDistance;
import frc.robot.autonomous.commands.RevThenShoot;
import frc.robot.autonomous.commands.TurnToTarget;
import frc.robot.subsystems.Drivebase;

public class ShootAndMove extends SequentialCommandGroup {

    public ShootAndMove(Drivebase drive, Indexer indexer, Shooter shooter, Limelight limelight, int pipeline) {

        addCommands(
            new TurnToTarget(drive, limelight, pipeline),
            new RevThenShoot(indexer, shooter).withTimeout(8),
            new DriveByDistance(drive, -1)
        );
    }
}