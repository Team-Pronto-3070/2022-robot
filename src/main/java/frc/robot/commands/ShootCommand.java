package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class ShootCommand extends SequentialCommandGroup{
    public ShootCommand(Drive_s drive, Shooter_s shooter, Indexer_s indexer){
        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> shooter.setRPM(), shooter),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new InstantCommand(() -> indexer.set(1), indexer),
                    new WaitCommand(3),
                    new InstantCommand(() -> indexer.stop(), indexer)
        )));
    }
}
