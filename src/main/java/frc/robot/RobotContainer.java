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
import frc.robot.commands.Auto_1BallNoMove;
import frc.robot.commands.Auto_2Ballv1;
import frc.robot.commands.Auto_2Ballv2;
import frc.robot.commands.Auto_2Ballv3;
import frc.robot.commands.Auto_Simple0Ball;
import frc.robot.commands.Auto_Simple1Ball;
import frc.robot.commands.Auto_Trajectory0Ball;
import frc.robot.commands.Auto_Trajectory1Ball;
import frc.robot.commands.Auto_TrajectoryTest;
import frc.robot.commands.ClearShooterCommand;
import frc.robot.commands.HighShootCommand;
import frc.robot.commands.Intake_DownCommand;
import frc.robot.commands.Intake_UpCommand;
import frc.robot.commands.LowShootCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.Climber_s;
import frc.robot.subsystems.Drive_s;
import frc.robot.subsystems.Indexer_s;
import frc.robot.subsystems.IntakeExtender_s;
import frc.robot.subsystems.Intake_s;
import frc.robot.subsystems.Shooter_s;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

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
  private final Intake_s intake = new Intake_s();
  private final Climber_s climber = new Climber_s();
  private final IntakeExtender_s intakeExtender = new IntakeExtender_s();

  private UsbCamera camera;

  private enum autoOptions {NONE, TRAJECTORY_TEST, SIMPLE_1_BALL, TRAJECTORY_1_BALL,
            SIMPLE_0_BALL, TRAJECTORY_0_BALL, NO_MOVE_1_BALL, V1_2_BALL, V2_2_BALL, V3_2_BALL}

  //define a sendable chooser to select the autonomous command
  private SendableChooser<autoOptions> autoChooser = new SendableChooser<autoOptions>();

  private Trigger intakeTrigger;
  private Trigger intakeReverseTrigger;
  private Trigger rightStickTrigger;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NetworkTableInstance.getDefault().setUpdateRate(0.01);
    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(160, 120);
    camera.setFPS(30);

    //add options to the chooser
    autoChooser.setDefaultOption("None", autoOptions.NONE);
    autoChooser.addOption("Simple test", autoOptions.TRAJECTORY_TEST);
    autoChooser.addOption("dead-reckoning 1 ball", autoOptions.SIMPLE_1_BALL);
    autoChooser.addOption("trajectory 1 ball", autoOptions.TRAJECTORY_1_BALL);
    autoChooser.addOption("NOSHOOTING dead-reckoning 0 ball", autoOptions.SIMPLE_0_BALL);
    autoChooser.addOption("NOSHOOTING trajectory 0 ball", autoOptions.TRAJECTORY_0_BALL);
    autoChooser.addOption("1 ball no move", autoOptions.NO_MOVE_1_BALL);
    autoChooser.addOption("2 ball v1", autoOptions.V1_2_BALL);
    autoChooser.addOption("2 ball v2", autoOptions.V2_2_BALL);
    autoChooser.addOption("2 ball v3", autoOptions.V3_2_BALL);

    //put the chooser on the dashboard
    SmartDashboard.putData(autoChooser);

    SmartDashboard.putNumber("high shooter rpm", Constants.SHOOTER.HIGH_RPM);
    SmartDashboard.putNumber("low shooter rpm", Constants.SHOOTER.LOW_RPM);

    drive.setDefaultCommand(new TeleopCommand(drive, oi));
    shooter.setDefaultCommand(new RunCommand(shooter::stop, shooter));
    indexer.setDefaultCommand(new RunCommand(indexer::stop, indexer));
    intake.setDefaultCommand(new RunCommand(intake::stop, intake));
    climber.setDefaultCommand(new RunCommand(climber::stop, climber));
    intakeExtender.setDefaultCommand(new RunCommand(intakeExtender::stop, intakeExtender));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //intakeTrigger = new Trigger(() -> oi.intakeSpeed.get() > Constants.INDEXER.DEADZONE);
    intakeReverseTrigger = new Trigger(() -> oi.intakeReverseSpeed.get() > Constants.INDEXER.DEADZONE);
    rightStickTrigger = new Trigger(() -> Math.abs(oi.rightStick.get()) > Constants.INDEXER.DEADZONE);

    //intakeTrigger.whileActiveContinuous(() -> intake.setSpeed(oi.intakeSpeed.get() * 0.5), intake);
    intakeReverseTrigger.whileActiveContinuous(() -> intake.setSpeed(-oi.intakeReverseSpeed.get() * 0.5), intake);
    
    oi.intakeExtenderManualButton.and(rightStickTrigger)
          .whileActiveContinuous(() -> intakeExtender.setExtenderSpeed(oi.rightStick.get()), intakeExtender);
    oi.climberManualButton.and(rightStickTrigger)
          .whileActiveContinuous(() -> climber.set(oi.rightStick.get()), climber);

    oi.smartIntakeButton1.whileActiveContinuous(
                () -> {
                  intake.forward();
                  indexer.set(indexer.indexerMiddleSwitch.get() ? 0 : 1);
                }, intake, indexer)
        .whenActive(new Intake_DownCommand(intakeExtender).withInterrupt(oi.overrideButton::get))
        .whenInactive(new Intake_UpCommand(intakeExtender).withInterrupt(() -> oi.overrideButton.get() || oi.smartIntakeButton2.get()));

    oi.smartIntakeButton2
          .whileActiveContinuous(
                () -> {
                  intake.forward();
                  indexer.set(indexer.indexerMiddleSwitch.get() && indexer.getHighSwitchLatch() ? 0 : 1);
                  shooter.set(indexer.getHighSwitchLatch() ? 0 : 0.15);
                }, intake, indexer, shooter)
        .whenActive(new Intake_DownCommand(intakeExtender).withInterrupt(oi.overrideButton::get))
        .whenInactive(new Intake_UpCommand(intakeExtender).withInterrupt(oi.overrideButton::get));

    oi.smartIndexerButton.whileHeld(() -> {
                  indexer.set(1);
                  intake.forward();
              }, indexer);
    oi.highSmartShooterButton.whenPressed(new HighShootCommand(drive, shooter, indexer)
                      .beforeStarting(new ClearShooterCommand(indexer, shooter))
                      .withInterrupt(oi.overrideButton::get));
    oi.lowSmartShooterButton.whenPressed(new LowShootCommand(drive, shooter, indexer)
                      .beforeStarting(new ClearShooterCommand(indexer, shooter))
                      .withInterrupt(oi.overrideButton::get));
    oi.indexerReverseButton.whileHeld(() -> indexer.set(-1), indexer);

    oi.intakeUpButton.whenActive(new Intake_UpCommand(intakeExtender).withInterrupt(oi.overrideButton::get));
    oi.intakeDownButton.whenActive(new Intake_DownCommand(intakeExtender).withInterrupt(oi.overrideButton::get));
    
    oi.clearShooterButton.whenActive(new ClearShooterCommand(indexer, shooter).withInterrupt(oi.overrideButton::get));
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
                                Map.entry(autoOptions.SIMPLE_0_BALL, new Auto_Simple0Ball(drive, shooter, indexer)),
                                Map.entry(autoOptions.TRAJECTORY_0_BALL, new Auto_Trajectory0Ball(drive, shooter, indexer)),
                                Map.entry(autoOptions.NO_MOVE_1_BALL, new Auto_1BallNoMove(drive, shooter, indexer)),
                                Map.entry(autoOptions.V1_2_BALL, new Auto_2Ballv1(drive, shooter, indexer)),
                                Map.entry(autoOptions.V2_2_BALL, new Auto_2Ballv2(drive, shooter, indexer)),
                                Map.entry(autoOptions.V3_2_BALL, new Auto_2Ballv3(drive, shooter, indexer))
                    ), autoChooser::getSelected);
  }
}
