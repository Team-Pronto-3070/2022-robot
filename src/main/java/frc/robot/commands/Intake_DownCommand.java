package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeExtender_s;

public class Intake_DownCommand extends SequentialCommandGroup {
    public Intake_DownCommand(IntakeExtender_s intakeExtender) {
        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> intakeExtender.setExtenderSpeed(-0.75), intakeExtender),
                new WaitUntilCommand(() -> intakeExtender.getExtenderPosition() <= 1.3)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> intakeExtender.setExtenderSpeed(-0.2), intakeExtender),
                new WaitUntilCommand(() -> intakeExtender.getExtenderPosition() <= Constants.INTAKE.DOWN_POSITION)
            ),
            new InstantCommand(() -> intakeExtender.setExtenderSpeed(0), intakeExtender)
        );
    }
}
