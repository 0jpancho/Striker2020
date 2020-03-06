package frc.robot.autonomous.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.commands.RevThenShoot;
import frc.robot.autonomous.commands.TurnToTarget;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;

public class ShootOnly extends SequentialCommandGroup  {

    public ShootOnly(Drivebase drive, Indexer indexer, Shooter shooter, Limelight limelight, int pipeline) {

        addCommands(
            new TurnToTarget(drive, limelight, pipeline),
            new RevThenShoot(indexer, shooter).withTimeout(8)
        );
    }
}