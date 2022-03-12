package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class Auto_2Ballv1 extends SequentialCommandGroup {
    public Auto_2Ballv1(Drive_s drive, Shooter_s shooter, Indexer_s indexer) {
        Trajectory outOfTarmac =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(1.5, 0, new Rotation2d(0)),
                drive.getTrajectoryConfig());
        
        addCommands(
            new LowShootCommand(drive, shooter, indexer),
            new InstantCommand(() -> drive.resetPose(new Pose2d()), drive), 
            new ProntoRamseteCommand(outOfTarmac, drive),
            new InstantCommand(() -> drive.stop(), drive),
            new ParallelRaceGroup(
                new RunCommand(() -> indexer.set(1), indexer),
                new RunCommand(() -> drive.tankDriveVolts(2, 2), drive),
                new WaitUntilCommand(indexer.indexerSwitchTrigger::get),
                new WaitCommand(5)
            ),
            new InstantCommand(indexer::stop, indexer),
            new InstantCommand(drive::stop, drive),
            new HighShootCommand(drive, shooter, indexer)
        );
    }
}
