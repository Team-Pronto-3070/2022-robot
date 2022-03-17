package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake_s extends SubsystemBase {

    private WPI_TalonSRX tal_Intake;
    private WPI_TalonSRX tal_Extender;

    private Encoder extenderEncoder;

    private ArmFeedforward extenderFF;
    private ProfiledPIDController extenderPID;

    public Intake_s() {

        tal_Intake = new WPI_TalonSRX(Constants.INTAKE.TAL_INTAKE_ID);
        tal_Intake.configFactoryDefault();
        tal_Intake.setNeutralMode(NeutralMode.Brake);
        tal_Intake.setInverted(false);
        tal_Intake.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);

        tal_Extender = new WPI_TalonSRX(Constants.INTAKE.TAL_EXTENDER_ID);
        tal_Extender.configFactoryDefault();
        tal_Extender.setNeutralMode(NeutralMode.Brake);
        tal_Extender.setInverted(false);
        tal_Extender.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);

        extenderEncoder = new Encoder(Constants.INTAKE.ENCODER_PORTS[0], Constants.INTAKE.ENCODER_PORTS[1]);
        extenderEncoder.setDistancePerPulse(Constants.INTAKE.ENCODER_DISTANCE_PER_PULSE); //units: radians
        extenderEncoder.reset();

        extenderFF = new ArmFeedforward(Constants.INTAKE.EXTENDER_FEEDFORWARD.S, Constants.INTAKE.EXTENDER_FEEDFORWARD.G, Constants.INTAKE.EXTENDER_FEEDFORWARD.V, Constants.INTAKE.EXTENDER_FEEDFORWARD.A);
        extenderPID = new ProfiledPIDController(Constants.INTAKE.EXTENDER_PID.P, Constants.INTAKE.EXTENDER_PID.I, Constants.INTAKE.EXTENDER_PID.D,
                        new TrapezoidProfile.Constraints(Constants.INTAKE.MAX_VELOCITY, Constants.INTAKE.MAX_ACCELERATION));
        extenderPID.reset(Constants.INTAKE.UP_POSITION);
        extenderPID.setGoal(Constants.INTAKE.UP_POSITION);
    }

    public void stop() {
        tal_Intake.set(0);
        tal_Extender.set(0);
    }

    public void forward() {
        tal_Intake.set(Constants.INTAKE.FORWARD_SPEED);
    }

    public void reverse() {
        tal_Intake.set(Constants.INTAKE.REVERSE_SPEED);
    }

    public void up() {
        extenderPID.setGoal(Constants.INTAKE.UP_POSITION);
    }

    public void down() {
        extenderPID.setGoal(Constants.INTAKE.DOWN_POSITION);
    }

    public void resetEncoder() {
        extenderEncoder.reset();
    }

    private double getExtenderPosition() {
        return extenderEncoder.getDistance() + Constants.INTAKE.UP_POSITION;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake extender position", extenderEncoder.getDistance());
        tal_Extender.setVoltage(extenderPID.calculate(getExtenderPosition())
                    + extenderFF.calculate(extenderPID.getSetpoint().position,
                                           extenderPID.getSetpoint().velocity));
    }
}
