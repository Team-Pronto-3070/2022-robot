package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber_s extends SubsystemBase{
    
    WPI_TalonSRX telescopingMotor;
    WPI_TalonSRX rotatingMotor;

    public Climber_s() {
        telescopingMotor = new WPI_TalonSRX(Constants.CLIMBER.TELESCOPING_MOTOR_ID);
        telescopingMotor.configFactoryDefault();
        telescopingMotor.setNeutralMode(NeutralMode.Brake);
        telescopingMotor.setInverted(false);

        rotatingMotor = new WPI_TalonSRX(Constants.CLIMBER.ROTATING_MOTOR_ID);
        rotatingMotor.configFactoryDefault();
        rotatingMotor.setNeutralMode(NeutralMode.Brake);
        rotatingMotor.setInverted(false);
    }

    public void stop() {
        telescopingMotor.set(0);
        rotatingMotor.set(0);
    }

    public void setTelescopingMotor(double speed) {
        telescopingMotor.set(speed);
    }

    public void setRotatingMotor(double speed) {
        telescopingMotor.set(speed);
    }
}
