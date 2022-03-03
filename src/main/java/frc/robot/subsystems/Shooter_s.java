// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter_s extends SubsystemBase {

  private WPI_TalonFX tal_Shooter;
  private double shooterRPM = Constants.SHOOTER.DEFAULT_SHOOTER_RPM;

  private final LinearSystem<N1, N1, N1> shooter_plant;
  private final KalmanFilter<N1, N1, N1> shooter_observer;
  private final LinearQuadraticRegulator<N1, N1, N1> shooter_controller;
  private final LinearSystemLoop<N1, N1, N1> shooter_loop;
  

  public Shooter_s() {

    tal_Shooter = new WPI_TalonFX(Constants.SHOOTER.TAL_SHOOTER_ID);
    tal_Shooter.configFactoryDefault();
    tal_Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    tal_Shooter.setNeutralMode(NeutralMode.Coast);
    tal_Shooter.setInverted(true);
    tal_Shooter.configOpenloopRamp(Constants.SHOOTER.RAMP_TIME);
    tal_Shooter.configPeakOutputReverse(0);
    
    shooter_plant = LinearSystemId.identifyVelocitySystem(Constants.SHOOTER.FEEDFORWARD.V, Constants.SHOOTER.FEEDFORWARD.A);

    shooter_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),    
          shooter_plant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder data is
          0.020);
    
    shooter_controller =
      new LinearQuadraticRegulator<>(
          shooter_plant,
          VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease this to more heavily penalize state excursion, or make the controller behave more aggressively
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the controller less aggressive. 12 is a good starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using notifiers.

    shooter_loop =
      new LinearSystemLoop<>(shooter_plant, shooter_controller, shooter_observer, 12.0, 0.020);

    SmartDashboard.putNumber("shooter rpm", Constants.SHOOTER.DEFAULT_SHOOTER_RPM);
  }

  public void set(double speed) {
    tal_Shooter.set(speed);
  }

  public double getRPM() {
    return tal_Shooter.getSelectedSensorVelocity() * 600 / 2048;
  }

  public void setRPM(double rpm) {
    shooter_loop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(rpm)));
    shooter_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(getRPM())));
    shooter_loop.predict(0.020);
    tal_Shooter.setVoltage(shooter_loop.getU(0));
  }

  public void setRPM() {
    setRPM(shooterRPM);
  }

  public void setDashboardRPM() {
    shooterRPM = SmartDashboard.getNumber("shooter rpm", Constants.SHOOTER.DEFAULT_SHOOTER_RPM);
  }

  public void stop() {
    shooter_loop.setNextR(VecBuilder.fill(0));
    shooter_loop.correct(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(getRPM())));
    shooter_loop.predict(0.020);
    tal_Shooter.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current shooter rpm", getRPM());
  }
}