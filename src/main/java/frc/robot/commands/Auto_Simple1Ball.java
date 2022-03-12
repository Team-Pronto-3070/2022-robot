package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class Auto_Simple1Ball extends SequentialCommandGroup{
    public Auto_Simple1Ball(Drive_s drive, Shooter_s shooter, Indexer_s indexer) {
        addCommands(
            new LowShootCommand(drive, shooter, indexer),
            new ParallelRaceGroup(
                new RunCommand(() -> drive.tankDriveVolts(3, 3), drive),
                new WaitCommand(3)
            ),
            new InstantCommand(() -> drive.stop(), drive)
        );
    }
}
