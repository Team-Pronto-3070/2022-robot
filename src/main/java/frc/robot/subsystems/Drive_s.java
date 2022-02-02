package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Drive subsystem
 */
public class Drive_s extends SubsystemBase{
    
    private WPI_TalonFX talLF;
    private WPI_TalonFX talLB;
    private WPI_TalonFX talRF;
    private WPI_TalonFX talRB;

    private DifferentialDrive diffDrive;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;

    private PIDController LPID;
    private PIDController RPID;
    private SimpleMotorFeedforward ff;

    private TrajectoryConfig trajectoryConfig;
    
    
    public Drive_s() {

        talLF = new WPI_TalonFX(Constants.DRIVE.TAL_LF_ID);
        talLB = new WPI_TalonFX(Constants.DRIVE.TAL_LB_ID);
        talRF = new WPI_TalonFX(Constants.DRIVE.TAL_RF_ID);
        talRB = new WPI_TalonFX(Constants.DRIVE.TAL_RB_ID);

        talLF.configFactoryDefault();
        talLB.configFactoryDefault();
        talRF.configFactoryDefault();
        talRB.configFactoryDefault();

        talLF.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        talRF.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        talLB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        talRB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        talLF.setNeutralMode(NeutralMode.Brake);
        talLB.setNeutralMode(NeutralMode.Brake);
        talRF.setNeutralMode(NeutralMode.Brake);
        talRB.setNeutralMode(NeutralMode.Brake);

        talLF.setInverted(false);
        talLB.setInverted(false);
        talRF.setInverted(true);
        talRB.setInverted(true);

        talLB.follow(talLF);
        talRB.follow(talRF);

        LPID = new PIDController(Constants.DRIVE.LPID.P,
                                 Constants.DRIVE.LPID.I,
                                 Constants.DRIVE.LPID.D);

        RPID = new PIDController(Constants.DRIVE.RPID.P,
                                 Constants.DRIVE.RPID.I,
                                 Constants.DRIVE.RPID.D);

        ff = new SimpleMotorFeedforward(Constants.DRIVE.FEEDFORWARD.ks,
                                        Constants.DRIVE.FEEDFORWARD.kv,
                                        Constants.DRIVE.FEEDFORWARD.ka);

        diffDrive = new DifferentialDrive(talLF, talRF);
        kinematics = new DifferentialDriveKinematics(Constants.DRIVE.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        trajectoryConfig = new TrajectoryConfig(Constants.DRIVE.MAX_VELOCITY,
                                                Constants.DRIVE.MAX_ACCELERATION)
                            .setKinematics(kinematics)
                            .addConstraint(new DifferentialDriveVoltageConstraint(ff, kinematics, 10));
    }

    /**
     * Arcade drive mode for the robot
     * @param forwardSpeed
     * @param rotation
     */
    public void arcadeDrive(double forwardSpeed, double rotation) {
        diffDrive.arcadeDrive(forwardSpeed, rotation);
    }

    public void curvatureDrive(double speed, double rotation) {
        diffDrive.curvatureDrive(speed, rotation, true);
    }

    public void tankDriveVolts(double left, double right) {
        talLF.setVoltage(left);
        talRF.setVoltage(right);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(Constants.DRIVE.SENSOR_VELOCITY_COEFFICIENT * talLF.getSelectedSensorVelocity(),
                                                Constants.DRIVE.SENSOR_VELOCITY_COEFFICIENT * talRF.getSelectedSensorVelocity());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(newPose, gyro.getRotation2d());
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public PIDController getLPID() {
        return LPID;
    }

    public PIDController getRPID() {
        return RPID;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return ff;
    }

    public TrajectoryConfig getTrajectoryConfig() {
        return trajectoryConfig;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("talLF", talLF.get());
        SmartDashboard.putNumber("talLR", talLB.get());
        SmartDashboard.putNumber("talBF", talRF.get());
        SmartDashboard.putNumber("talBR", talRB.get());

        odometry.update(gyro.getRotation2d(),
                        Constants.DRIVE.SENSOR_POSITION_COEFFICIENT * talLF.getSelectedSensorPosition(),
                        Constants.DRIVE.SENSOR_POSITION_COEFFICIENT * talRF.getSelectedSensorPosition());
    }
}
