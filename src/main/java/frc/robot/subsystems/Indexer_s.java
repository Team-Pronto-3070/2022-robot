// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

/**
 * Shooter Subsystem
 * 
 * Handles all the motors in the shooter
 */
public class Indexer_s extends SubsystemBase {

  private WPI_TalonSRX tal_Indexer;

  public Indexer_s() {

    tal_Indexer = new WPI_TalonSRX(Constants.INDEXER.TAL_INDEXER_ID);
    tal_Indexer.configFactoryDefault();
    tal_Indexer.setNeutralMode(NeutralMode.Brake);
    tal_Indexer.setInverted(false);
    tal_Indexer.configOpenloopRamp(Constants.INDEXER.RAMP_TIME);
  }
  
  public void set(double speed) {
    tal_Indexer.set(speed);
  }

  public void stop() {
    tal_Indexer.set(0);
  }
}
