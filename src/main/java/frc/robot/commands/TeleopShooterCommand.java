package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Shooter_s;

/**
 * Default shooter command
 */
public class TeleopShooterCommand extends CommandBase {

    private final Shooter_s _shooter;
    private final OI _oi;

    public TeleopShooterCommand(Shooter_s shooter, OI oi) {

        _shooter = shooter;
        _oi = oi;

        addRequirements(_shooter);
    }

    //Overrides execute & periodically sends input to shooter
    @Override
    public void execute() {
        // TODO: Determine OI for shooter/intake/indexer

        _shooter.setIntake(
            ((_oi.IntakeButton.getAsBoolean()) ? Constants.SHOOTER.INTAKE_SPEED : 0)
        );

        _shooter.setIndexer(
            ((_oi.IndexerButton.getAsBoolean()) ? Constants.SHOOTER.INDEXER_SPEED : 0)
        );

        closedLoop(
            ((_oi.ShooterButton.getAsBoolean()) ? Constants.SHOOTER.SHOOTER_SETPOINT : 0)
        );

    }

    /**
     * calculates the speed of the shooting motor
     * @param shooterSpeedSetpoint - The target speed of the motor
     */
    private void closedLoop(double shooterSpeedSetpoint) {

        SmartDashboard.putNumber("SH_SETPOINT", shooterSpeedSetpoint);

        BangBangController bb = new BangBangController();

        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.SHOOTER.FEEDFORWARD.ks,
                                            Constants.SHOOTER.FEEDFORWARD.kv,
                                            Constants.SHOOTER.FEEDFORWARD.ka); 

        var m_shooterController = bb;

        final double shooterFeedforward =
            ff.calculate(
                shooterSpeedSetpoint);

        double shooterOutput =
            0.9 * shooterFeedforward
                + m_shooterController.calculate(
                    _shooter.getShooterSpeed(), shooterSpeedSetpoint);

        _shooter.setShooter(shooterOutput);
    }
}
