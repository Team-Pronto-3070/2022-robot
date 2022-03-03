package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class Auto_Simple1Ball extends SequentialCommandGroup{
    public Auto_Simple1Ball(Drive_s drive, Shooter_s shooter, Indexer_s indexer) {
        addCommands(
            new LowShootCommand(drive, shooter, indexer),
            new InstantCommand(() -> drive.tankDriveVolts(6, 6), drive),
            new WaitCommand(5),
            new InstantCommand(() -> drive.stop(), drive)
        );
    }
}
