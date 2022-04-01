package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeExtender_s;

public class Intake_UpCommand extends SequentialCommandGroup {
    public Intake_UpCommand(IntakeExtender_s intakeExtender) {
        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> intakeExtender.setExtenderSpeed(1), intakeExtender),
                new WaitUntilCommand(() -> intakeExtender.getExtenderPosition() >= -0.25)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> intakeExtender.setExtenderSpeed(0.5), intakeExtender),
                new WaitUntilCommand(() -> intakeExtender.getExtenderPosition() >= 0.8)
            ),
            new ParallelRaceGroup(
                new RunCommand(() -> intakeExtender.setExtenderSpeed(0.25), intakeExtender),
                new WaitUntilCommand(() -> intakeExtender.getExtenderPosition() >= Constants.INTAKE.UP_POSITION)
            ),
            new InstantCommand(() -> intakeExtender.setExtenderSpeed(0), intakeExtender)
        );
    }
}
