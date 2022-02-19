package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Shooter_s;

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

        _shooter.setIntake(
            0
            //((_oi.IntakeButton.isPressed()) ? Constants.SHOOTER.SETPOINT : 0)// TODO
        );

        closedLoop(
            0
            //((_oi.ShooterButton.isPressed()) ? Constants.SHOOTER.SETPOINT : 0)// TODO
        );

    }

    
  private void closedLoop(double shooterSpeedSetpoint) {

    SmartDashboard.putNumber("SH_SETPOINT", shooterSpeedSetpoint);

    BangBangController bb = new BangBangController();

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.SHOOTER.FEEDFORWARD.ks,
                                        Constants.SHOOTER.FEEDFORWARD.kv,
                                        Constants.SHOOTER.FEEDFORWARD.ka); 


    double shooterOutput;

    var m_shooterController = bb;

    final double shooterFeedforward =
          ff.calculate(
              shooterSpeedSetpoint);

    shooterOutput =
          .9 * shooterFeedforward
              + m_shooterController.calculate(
                  _shooter.getShooterSpeed(), shooterSpeedSetpoint);

    _shooter.setShooter(shooterOutput);
  }

    
}
