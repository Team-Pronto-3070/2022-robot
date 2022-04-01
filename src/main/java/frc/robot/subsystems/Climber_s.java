package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber_s extends SubsystemBase{
    
    private WPI_TalonSRX talFront;
    private WPI_TalonSRX talBack;

    public DigitalInput limitSwitch;
    
    public Climber_s() {
        talFront = new WPI_TalonSRX(Constants.CLIMBER.TAL_FRONT_ID);
        talBack = new WPI_TalonSRX(Constants.CLIMBER.TAL_BACK_ID);
        talFront.configFactoryDefault();
        talBack.configFactoryDefault();
        talFront.setNeutralMode(NeutralMode.Brake);
        talBack.setNeutralMode(NeutralMode.Brake);
        talFront.setInverted(true);
        talBack.setInverted(true);
        talFront.configOpenloopRamp(Constants.CLIMBER.RAMP_TIME);
        talBack.configOpenloopRamp(Constants.CLIMBER.RAMP_TIME);

        talBack.follow(talFront);

        limitSwitch = new DigitalInput(Constants.CLIMBER.LIMIT_SWITCH_PORT);
    }

    public void stop() {
        talFront.set(0);
    }

    public void set(double speed) {
        talFront.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("climber limit switch", limitSwitch.get());
    }
}
