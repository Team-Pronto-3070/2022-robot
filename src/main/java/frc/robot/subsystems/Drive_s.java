package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SPI;
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

    private ADIS16448_IMU gyro;

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

        talLF.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);
        talLB.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);
        talRF.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);
        talRB.configOpenloopRamp(Constants.DRIVE.RAMP_TIME);

        talLB.follow(talLF);
        talRB.follow(talRF);

        gyro = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kX, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);

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
        odometry = new DifferentialDriveOdometry(getGyro());

        trajectoryConfig = new TrajectoryConfig(Constants.DRIVE.MAX_VELOCITY,
                                                Constants.DRIVE.MAX_ACCELERATION)
                            .setKinematics(kinematics)
                            .addConstraint(new DifferentialDriveVoltageConstraint(ff, kinematics, 10))
                            .addConstraint(new CentripetalAccelerationConstraint(Constants.DRIVE.MAX_CENTRIPETAL_ACCELERATION));
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
        diffDrive.feed();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(Constants.DRIVE.SENSOR_VELOCITY_COEFFICIENT * talLF.getSelectedSensorVelocity(),
                                                Constants.DRIVE.SENSOR_VELOCITY_COEFFICIENT * talRF.getSelectedSensorVelocity());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyro() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetPose(Pose2d newPose) {
        talLF.setSelectedSensorPosition(0);
        talRF.setSelectedSensorPosition(0);
        odometry.resetPosition(newPose, getGyro());
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

    public void stop() {
        talLF.set(0);
        talRF.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("talLF", talLF.get());
        SmartDashboard.putNumber("talLB", talLB.get());
        SmartDashboard.putNumber("talRF", talRF.get());
        SmartDashboard.putNumber("talRB", talRB.get());

        SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putNumber("gyro_pos_X", gyro.getGyroAngleX());
        SmartDashboard.putNumber("gyro_pos_Y", gyro.getGyroAngleY());
        SmartDashboard.putNumber("gyro_pos_Z", gyro.getGyroAngleZ());
        SmartDashboard.putNumber("gyro_vel_X", gyro.getGyroRateX());
        SmartDashboard.putNumber("gyro_vel_Y", gyro.getGyroRateY());
        SmartDashboard.putNumber("gyro_vel_Z", gyro.getGyroRateZ());

        SmartDashboard.putNumber("pose_x", getPose().getX());
        SmartDashboard.putNumber("pose_y", getPose().getY());
        SmartDashboard.putString("pose_theta", getPose().getRotation().toString());

        odometry.update(getGyro(),
                        Constants.DRIVE.SENSOR_POSITION_COEFFICIENT * talLF.getSelectedSensorPosition(),
                        Constants.DRIVE.SENSOR_POSITION_COEFFICIENT * talRF.getSelectedSensorPosition());
    }
}
