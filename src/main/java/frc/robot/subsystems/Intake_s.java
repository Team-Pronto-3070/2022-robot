package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake_s extends SubsystemBase {

    private WPI_TalonSRX tal_Intake;

    public Intake_s() {

        tal_Intake = new WPI_TalonSRX(Constants.INTAKE.TAL_INTAKE_ID);
        tal_Intake.configFactoryDefault();
        tal_Intake.setNeutralMode(NeutralMode.Coast);
        tal_Intake.setInverted(true);
        tal_Intake.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);
    }

    public void stop() {
        tal_Intake.set(0);
    }

    public void setSpeed(double speed) {
        tal_Intake.set(speed);
    }

    public void forward() {
        tal_Intake.set(Constants.INTAKE.FORWARD_SPEED);
    }

    public void reverse() {
        tal_Intake.set(-Constants.INTAKE.REVERSE_SPEED);
    }
}
