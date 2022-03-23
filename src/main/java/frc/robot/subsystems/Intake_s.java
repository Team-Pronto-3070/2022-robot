package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake_s extends SubsystemBase {

    private WPI_TalonSRX tal_Intake;
    private WPI_TalonSRX tal_Extender;

    private DutyCycleEncoder extenderAbsoluteEncoder;
/*
    private ArmFeedforward extenderFF;
    private ProfiledPIDController extenderPID;
*/
    public Intake_s() {

        tal_Intake = new WPI_TalonSRX(Constants.INTAKE.TAL_INTAKE_ID);
        tal_Intake.configFactoryDefault();
        tal_Intake.setNeutralMode(NeutralMode.Coast);
        tal_Intake.setInverted(true);
        tal_Intake.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);

        tal_Extender = new WPI_TalonSRX(Constants.INTAKE.TAL_EXTENDER_ID);
        tal_Extender.configFactoryDefault();
        tal_Extender.setNeutralMode(NeutralMode.Brake);
        tal_Extender.setInverted(false);
        tal_Extender.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);

        extenderAbsoluteEncoder = new DutyCycleEncoder(Constants.INTAKE.ENCODER_PORTS[3]);
        extenderAbsoluteEncoder.setDistancePerRotation(2 * Math.PI);
        extenderAbsoluteEncoder.setPositionOffset(Constants.INTAKE.HORIZONTAL_POSITION_OFFSET);
/*
        extenderFF = new ArmFeedforward(Constants.INTAKE.EXTENDER_FEEDFORWARD.S, Constants.INTAKE.EXTENDER_FEEDFORWARD.G, Constants.INTAKE.EXTENDER_FEEDFORWARD.V, Constants.INTAKE.EXTENDER_FEEDFORWARD.A);
        extenderPID = new ProfiledPIDController(Constants.INTAKE.EXTENDER_PID.P, Constants.INTAKE.EXTENDER_PID.I, Constants.INTAKE.EXTENDER_PID.D,
                        new TrapezoidProfile.Constraints(Constants.INTAKE.MAX_VELOCITY, Constants.INTAKE.MAX_ACCELERATION));
        extenderPID.reset(getExtenderPosition());
        extenderPID.setGoal(Constants.INTAKE.UP_POSITION);
        */
    }

    public void stop() {
        tal_Intake.set(0);
        tal_Extender.set(0);
    }

    public void setSpeed(double speed) {
        tal_Intake.set(speed);
    }

    public void setExtenderSpeed(double speed) {
        tal_Extender.set(speed);
    }

    public void forward() {
        tal_Intake.set(Constants.INTAKE.FORWARD_SPEED);
    }

    public void reverse() {
        tal_Intake.set(-Constants.INTAKE.REVERSE_SPEED);
    }

/*
    public void up() {
        extenderPID.setGoal(Constants.INTAKE.UP_POSITION);
    }

    public void down() {
        extenderPID.setGoal(Constants.INTAKE.DOWN_POSITION);
    }
*/

    public double getExtenderPosition() {
        return extenderAbsoluteEncoder.getDistance();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake extender absolute position", getExtenderPosition());
        SmartDashboard.putNumber("extender motor percent", tal_Extender.get());
        /*
        if (!extenderPID.atGoal()) {
            SmartDashboard.putBoolean("extender at goal", false);
            tal_Extender.set(extenderPID.calculate(getExtenderPosition()) / 12
                        + extenderFF.calculate(extenderPID.getSetpoint().position,
                                            extenderPID.getSetpoint().velocity));
            SmartDashboard.putNumber("extender pid calculated voltage", extenderPID.calculate(getExtenderPosition()));
        } else {
            SmartDashboard.putBoolean("extender at goal", true);
            tal_Extender.set(0);
        }
        SmartDashboard.putNumber("extender motor voltage", tal_Extender.getMotorOutputVoltage());
        SmartDashboard.putNumber("extender motor percent", tal_Extender.get());
        SmartDashboard.putNumber("extender PID goal position", extenderPID.getGoal().position);
        SmartDashboard.putNumber("extender PID setpoint position", extenderPID.getSetpoint().position);
        */
    }
}
