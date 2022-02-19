// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class Shooter_s extends SubsystemBase {

  private WPI_TalonFX tal_Shooter;
  private WPI_TalonSRX tal_Indexer;
  private WPI_TalonSRX tal_Intake;
  
  public Shooter_s() {

    tal_Shooter = new WPI_TalonFX(Constants.SHOOTER.TAL_SH_ID);
    tal_Indexer = new WPI_TalonSRX(Constants.SHOOTER.TAL_ID_ID);
    tal_Intake = new WPI_TalonSRX(Constants.SHOOTER.TAL_IN_ID);
    
    tal_Shooter.configFactoryDefault();
    tal_Indexer.configFactoryDefault();
    tal_Intake.configFactoryDefault();

    tal_Shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    tal_Shooter.setNeutralMode(NeutralMode.Coast);
    tal_Indexer.setNeutralMode(NeutralMode.Brake);
    tal_Intake.setNeutralMode(NeutralMode.Brake);

    tal_Shooter.setInverted(false);
    tal_Indexer.setInverted(false);
    tal_Intake.setInverted(false);

    tal_Shooter.configOpenloopRamp(Constants.SHOOTER.RAMP_TIME);
    tal_Indexer.configOpenloopRamp(Constants.SHOOTER.RAMP_TIME);
    tal_Intake.configOpenloopRamp(Constants.SHOOTER.RAMP_TIME);                          
  }

  public void setIndexer(double speed) {
    tal_Indexer.set(speed);
  }

  public void setShooter(double speed) {
    tal_Shooter.set(speed);
  }

  public void setIntake(double speed) {
    tal_Intake.set(speed);
  }

  public double getShooterSpeed() {
    return tal_Shooter.getSelectedSensorVelocity();
  }

  public void stop() {
    tal_Shooter.set(0);
    tal_Indexer.set(0);
    tal_Intake.set(0);
  }
}

