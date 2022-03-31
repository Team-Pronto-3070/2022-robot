package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class ClearShooterCommand extends SequentialCommandGroup{
    
    public ClearShooterCommand(Indexer_s indexer, Shooter_s shooter) {
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(indexer::resetHighSwitchLatch, indexer),
                    new InstantCommand(shooter::enableReverse, shooter),
                    new ParallelRaceGroup(
                        new RunCommand(() -> indexer.set(-1), indexer),
                        new RunCommand(() -> shooter.set(-0.2), shooter),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> !indexer.indexerMiddleSwitch.get()),
                            new WaitCommand(0.1),
                            new WaitUntilCommand(indexer.indexerMiddleSwitch::get)
                        )
                    ),
                    new InstantCommand(shooter::stop, shooter),
                    new InstantCommand(indexer::stop, indexer),
                    new InstantCommand(shooter::disableReverse, shooter)
                ),
                new WaitCommand(0),
                indexer::getHighSwitchLatch
            ),
            new InstantCommand(indexer::resetHighSwitchLatch, indexer)
        );
    }

}
