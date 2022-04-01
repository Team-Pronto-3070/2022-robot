package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.IntakeExtender_s;
import frc.robot.subsystems.Intake_s;
import frc.robot.subsystems.Shooter_s;

public class Auto_2Ballv4 extends SequentialCommandGroup {
    public Auto_2Ballv4(Drive_s drive, Shooter_s shooter, Indexer_s indexer, Intake_s intake, IntakeExtender_s intakeExtender) {
        Trajectory outOfTarmac =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                new Pose2d(2.5, 0, new Rotation2d(0)),
                drive.getTrajectoryConfig());
        
        Trajectory setUpForShot =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.5, 0, Rotation2d.fromDegrees(0)),
                List.of(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                drive.getTrajectoryConfig().setReversed(true));
        
        addCommands(
            new LowShootCommand(drive, shooter, indexer),
            new InstantCommand(() -> drive.resetPose(outOfTarmac.getInitialPose()), drive),
            new ParallelDeadlineGroup(
                //deadline
                //new WaitUntilCommand(indexer.indexerMiddleSwitch::get),

                //commands
                new ProntoRamseteCommand(outOfTarmac, drive),
                new Intake_DownCommand(intakeExtender),
                new RunCommand(intake::forward, intake),
                new RunCommand(() -> indexer.set(1), indexer).withInterrupt(indexer.indexerMiddleSwitch::get)
            ),
            new InstantCommand(indexer::stop, indexer),
            new InstantCommand(intake::stop, intake),
            new ParallelCommandGroup(
                new Intake_UpCommand(intakeExtender),
                new ProntoRamseteCommand(setUpForShot, drive)
            ),
            new InstantCommand(drive::stop, drive),
            new LowShootCommand(drive, shooter, indexer)
        );
    }
}
