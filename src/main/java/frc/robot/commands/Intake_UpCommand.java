package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake_s;

public class Intake_UpCommand extends SequentialCommandGroup {
    public Intake_UpCommand(Intake_s intake) {
        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> intake.setExtenderSpeed(1), intake),
                new WaitUntilCommand(() -> intake.getExtenderPosition() >= 1)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> intake.setExtenderSpeed(0.25), intake),
                new WaitUntilCommand(() -> intake.getExtenderPosition() >= Constants.INTAKE.UP_POSITION)
            ),
            new InstantCommand(() -> intake.setExtenderSpeed(0), intake)
        );
    }
}
