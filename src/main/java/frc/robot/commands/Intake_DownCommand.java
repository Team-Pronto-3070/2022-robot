package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake_s;

public class Intake_DownCommand extends SequentialCommandGroup {
    public Intake_DownCommand(Intake_s intake) {
        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> intake.setExtenderSpeed(-0.75), intake),
                new WaitUntilCommand(() -> intake.getExtenderPosition() <= 1.4)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> intake.setExtenderSpeed(-0.1), intake),
                new WaitUntilCommand(() -> intake.getExtenderPosition() <= Constants.INTAKE.DOWN_POSITION)
            ),
            new InstantCommand(() -> intake.setExtenderSpeed(0), intake)
        );
    }
}
