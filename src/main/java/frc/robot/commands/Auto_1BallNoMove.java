package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class Auto_1BallNoMove extends SequentialCommandGroup{
    public Auto_1BallNoMove(Drive_s drive, Shooter_s shooter, Indexer_s indexer) {
        addCommands(
            new LowShootCommand(drive, shooter, indexer)
        );
    }
}