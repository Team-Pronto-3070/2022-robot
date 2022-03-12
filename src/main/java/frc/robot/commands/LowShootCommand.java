package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class LowShootCommand extends SequentialCommandGroup{
    public LowShootCommand(Drive_s drive, Shooter_s shooter, Indexer_s indexer){
        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> shooter.setRPM(SmartDashboard.getNumber("low shooter rpm", Constants.SHOOTER.LOW_RPM)), shooter),
                new SequentialCommandGroup(
                    new WaitUntilCommand(shooter::atSetpoint),
                    new WaitCommand(2),
                    new InstantCommand(() -> indexer.set(1), indexer),
                    new WaitCommand(2),
                    new InstantCommand(() -> indexer.stop(), indexer)
                )
            ),
            new InstantCommand(() -> indexer.stop(), indexer),
            new InstantCommand(() -> shooter.stop(), shooter)
        );
    }
}