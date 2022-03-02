// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private double shooterRPM = Constants.SHOOTER.DEFAULT_SHOOTER_RPM;
  

  public Shooter_s() {

    tal_Shooter = new WPI_TalonFX(Constants.SHOOTER.TAL_SHOOTER_ID);
    tal_Shooter.configFactoryDefault();
    tal_Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    tal_Shooter.setNeutralMode(NeutralMode.Coast);
    tal_Shooter.setInverted(false);
    tal_Shooter.configOpenloopRamp(Constants.SHOOTER.RAMP_TIME);  
    
    shooter_bb = new BangBangController();

    SmartDashboard.putNumber("shooter rpm", Constants.SHOOTER.DEFAULT_SHOOTER_RPM);
  }

  public void set(double speed) {
    tal_Shooter.set(speed);
  }

  public double getRPM() {
    return tal_Shooter.getSelectedSensorVelocity() * 600 / 2048;
  }

  public void setRPM(double rpm) {
    set(shooter_bb.calculate(getRPM(), rpm));
  }

  public void setRPM() {
    setRPM(shooterRPM);
  }

  public void setDashboardRPM() {
    shooterRPM = SmartDashboard.getNumber("shooter rpm", Constants.SHOOTER.DEFAULT_SHOOTER_RPM);
  }

  public void stop() {
    tal_Shooter.set(0);
  }
}

