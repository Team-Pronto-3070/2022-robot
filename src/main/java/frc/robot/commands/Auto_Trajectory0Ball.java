package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;

public class Auto_Trajectory0Ball extends SequentialCommandGroup{
    public Auto_Trajectory0Ball(Drive_s drive, Shooter_s shooter, Indexer_s indexer) {
    
    // ~ 9.5 feet
    Trajectory outOfTarmac =
        //drive straight 3 meters
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            drive.getTrajectoryConfig());
    
    addCommands(
        new InstantCommand(() -> drive.resetPose(new Pose2d()), drive), 
        new ProntoRamseteCommand(outOfTarmac, drive),
        new InstantCommand(() -> drive.stop(), drive));
    }
}
