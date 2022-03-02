// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto_Simple0Ball;
import frc.robot.commands.Auto_Simple1Ball;
import frc.robot.commands.Auto_Trajectory1Ball;
import frc.robot.commands.Auto_TrajectoryTest;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.Shooter_s;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive_s drive = new Drive_s();
  private final OI oi = new OI();
  private final Shooter_s shooter = new Shooter_s();
  private final Indexer_s indexer = new Indexer_s();

  private enum autoOptions {NONE, TRAJECTORY_TEST, SIMPLE_1_BALL, TRAJECTORY_1_BALL, SIMPLE_0_BALL}

  //define a sendable chooser to select the autonomous command
  private SendableChooser<autoOptions> autoChooser = new SendableChooser<autoOptions>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    NetworkTableInstance.getDefault().setUpdateRate(0.01);

    //add options to the chooser
    autoChooser.setDefaultOption("None", autoOptions.NONE);
    autoChooser.addOption("Simple test", autoOptions.TRAJECTORY_TEST);
    autoChooser.addOption("dead-reckoning 1 ball", autoOptions.SIMPLE_1_BALL);
    autoChooser.addOption("trajectory 1 ball", autoOptions.TRAJECTORY_1_BALL);
    autoChooser.addOption("NOSHOOTING dead-reckoning 0 ball", autoOptions.SIMPLE_0_BALL);

    //put the chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    drive.setDefaultCommand(new TeleopCommand(drive, oi));
    shooter.setDefaultCommand(new RunCommand(shooter::stop, shooter));
    indexer.setDefaultCommand(new RunCommand(() -> {SmartDashboard.putBoolean("temp indexer troubleshooting", oi.indexerReverseButton.getAsBoolean());
                                                    SmartDashboard.putNumber("temp indexer troubleshooting 2",  oi.IndexerSpeed.get() * (oi.indexerReverseButton.get() ? -1 : 1));
                                                    SmartDashboard.putBoolean("xbox button 2", oi.xbox.getRawButton(2));
                                                    SmartDashboard.putBoolean("plain get", oi.indexerReverseButton.get());
                                                    
                                                    indexer.set(oi.IndexerSpeed.get() > Constants.INDEXER.DEADZONE ? 
                                                            oi.IndexerSpeed.get() * (oi.indexerReverseButton.get() ? -1 : 1) : 0);
                                                }, indexer));
//    indexer.setDefaultCommand(new RunCommand(() -> SmartDashboard.putBoolean("temp indexer troubleshooting", oi.indexerReverseButton.get()), indexer));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    oi.shooterButton.whileHeld(shooter::setRPM, shooter);
    oi.getDashboardShooterRPM.whenPressed(shooter::setDashboardRPM, shooter);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SelectCommand(Map.ofEntries(
                                Map.entry(autoOptions.NONE, new InstantCommand(() -> System.out.println("no auto command selected"))),
                                Map.entry(autoOptions.TRAJECTORY_TEST, new Auto_TrajectoryTest(drive)),
                                Map.entry(autoOptions.SIMPLE_1_BALL, new Auto_Simple1Ball(drive, shooter, indexer)),
                                Map.entry(autoOptions.TRAJECTORY_1_BALL, new Auto_Trajectory1Ball(drive, shooter, indexer)),
                                Map.entry(autoOptions.SIMPLE_0_BALL, new Auto_Simple0Ball(drive, shooter, indexer))
                    ), autoChooser::getSelected);
  }
}
