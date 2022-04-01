// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

/**
 * Shooter Subsystem
 * 
 * Handles all the motors in the shooter
 */
public class Shooter_s extends SubsystemBase {

  private WPI_TalonFX tal_Shooter;
  private BangBangController shooter_bb;
  private PIDController shooter_PID;
  private SimpleMotorFeedforward shooter_ff;

  private double setpoint = 0;

  public Shooter_s() {

    tal_Shooter = new WPI_TalonFX(Constants.SHOOTER.TAL_SHOOTER_ID);
    tal_Shooter.configFactoryDefault();
    tal_Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //tal_Shooter.setNeutralMode(NeutralMode.Coast);
    tal_Shooter.setNeutralMode(NeutralMode.Brake);
    tal_Shooter.setInverted(true);
    tal_Shooter.configOpenloopRamp(Constants.SHOOTER.RAMP_TIME);
    //tal_Shooter.configPeakOutputReverse(0);
    tal_Shooter.config_kP(0, Constants.SHOOTER.PID.P);
    tal_Shooter.config_kI(0, Constants.SHOOTER.PID.I);
    tal_Shooter.config_kD(0, Constants.SHOOTER.PID.D);
//    tal_Shooter.config_kF(0, Constants.SHOOTER.FEEDFORWARD.V * 0.9);
    
//    shooter_bb = new BangBangController();
//    shooter_PID = new PIDController(Constants.SHOOTER.PID.P, Constants.SHOOTER.PID.I, Constants.SHOOTER.PID.D);
    shooter_ff = new SimpleMotorFeedforward(Constants.SHOOTER.FEEDFORWARD.S, Constants.SHOOTER.FEEDFORWARD.V, Constants.SHOOTER.FEEDFORWARD.A);
  }

  public void set(double speed) {
    tal_Shooter.set(speed);
  }

  public double getRPM() {
    return tal_Shooter.getSelectedSensorVelocity() * 600.0 / 2048.0;
  }

  public void setRPM(double rpm) {
    setpoint = rpm;

//    set(shooter_bb.calculate(getRPM(), rpm));
//    set(MathUtil.clamp(shooter_PID.calculate(getRPM(), rpm), 0, 1));
//    tal_Shooter.set(ControlMode.Velocity, rpm * 2048 / 600,
//                    DemandType.ArbitraryFeedForward, Constants.SHOOTER.FEEDFORWARD.S * 0.9 / 12);
    
    tal_Shooter.set(ControlMode.Velocity, rpm * 2048.0 / 600.0);
    //tal_Shooter.setVoltage(shooter_ff.calculate(rpm));
  }

  public void stop() {
    setpoint = 0;
    tal_Shooter.set(0);
  }

  public Boolean atSetpoint() {
    return Math.abs(getRPM() - setpoint) < Constants.SHOOTER.RPM_TOLERANCE;
  }

  public void enableReverse() {
    tal_Shooter.configPeakOutputReverse(-1);
    tal_Shooter.setNeutralMode(NeutralMode.Brake);
  }

  public void disableReverse() {
    tal_Shooter.configPeakOutputReverse(0);
    tal_Shooter.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current shooter rpm", getRPM());
  }
}

