package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.OI;
import frc.robot.subsystems.Drive_s;

import frc.robot.Constants;

public class TeleopCommand extends CommandBase {

    private final Drive_s _drive;
    private final OI _oi;

    public TeleopCommand(Drive_s drive, OI oi) {
        _drive = drive;
        _oi = oi;

        addRequirements(_drive);
    }

    //Overrides execute & periodically sends input to drivetrain
    @Override
    public void execute() {
        SmartDashboard.putNumber("teleop_x", _oi.teleopX.get());
        SmartDashboard.putNumber("teleop_theta", _oi.teleopTheta.get());

        ///_drive.arcadeDrive(
        _drive.curvatureDrive(
            (Math.abs(_oi.teleopX.get()) < Constants.TELEOP_COMMAND.JOY_STICK_DEADZONE) ?
                    0 : _oi.teleopX.get() * (_oi.slowButton.get() ? Constants.TELEOP_COMMAND.SLOW_VX_COEFFICIENT : Constants.TELEOP_COMMAND.VX_COEFFICIENT), 
            (Math.abs(_oi.teleopTheta.get()) < Constants.TELEOP_COMMAND.JOY_STICK_OMEGA_DEADZONE) ?
                    0 : _oi.teleopTheta.get() * (_oi.slowButton.get() ? Constants.TELEOP_COMMAND.SLOW_OMEGA_COEFFICIENT : Constants.TELEOP_COMMAND.OMEGA_COEFFICIENT));
  }
}