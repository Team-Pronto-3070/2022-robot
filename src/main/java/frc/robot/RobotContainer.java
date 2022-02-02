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
import frc.robot.commands.Auto_SimpleTest;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.Drive_s;
import edu.wpi.first.wpilibj2.command.Command;
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

  private enum autoOptions {SIMPLE_TEST}

  //define a sendable chooser to select the autonomous command
  private SendableChooser<autoOptions> autoChooser = new SendableChooser<autoOptions>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    NetworkTableInstance.getDefault().setUpdateRate(0.01);

    //add options to the chooser
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Simple test", autoOptions.SIMPLE_TEST);

    //put the chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    drive.setDefaultCommand(new TeleopCommand(drive, oi));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SelectCommand(Map.ofEntries(
                                Map.entry(autoOptions.SIMPLE_TEST, new Auto_SimpleTest(drive))
                    ), autoChooser::getSelected);
  }
}
