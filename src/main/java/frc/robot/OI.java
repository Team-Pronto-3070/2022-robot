package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Framework for the operator interface for the robot.
 * 
 * Contains necessary sensors and manual control objects (i.e. joysticks).
 */
public class OI {

    public XboxController xbox;

    public final Supplier<Double> teleopX;
    public final Supplier<Double> teleopTheta;

    public final Supplier<Double> intakeSpeed;
    public final Supplier<Double> intakeReverseSpeed;
    public final Supplier<Double> extenderSpeed;

    public final Button smartIntakeButton;
    public final Button smartIndexerButton;
    public final Button highSmartShooterButton;
    public final Button lowSmartShooterButton;
    public final Button slowButton;
    public final Button overrideButton;
    public final Button indexerReverseButton;

    /**
     * Constructs the Operator Interface.
     */
    public OI() {    
        xbox = new XboxController(Constants.OI.XBOX_PORT);

        teleopX = () -> -xbox.getLeftY();
        teleopTheta = () -> xbox.getLeftX();

        intakeSpeed = () -> xbox.getLeftTriggerAxis();
        intakeReverseSpeed = () -> xbox.getRightTriggerAxis();
        extenderSpeed = () -> -xbox.getRightY();

        smartIntakeButton = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
        smartIndexerButton = new JoystickButton(xbox, XboxController.Button.kA.value);
        highSmartShooterButton = new JoystickButton(xbox, XboxController.Button.kX.value);
        lowSmartShooterButton = new JoystickButton(xbox, XboxController.Button.kY.value);
        slowButton = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
        overrideButton = new JoystickButton(xbox, XboxController.Button.kStart.value);
        indexerReverseButton = new JoystickButton(xbox, XboxController.Button.kB.value);
    }
}