package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Framework for the operator interface for the robot.
 * 
 * Contains necessary sensors and manual control objects (i.e. joysticks).
 */
public class OI {

    public XboxController xbox;

    public final Supplier<Double> teleopX;
    public final Supplier<Double> teleopTheta;

    //public final Supplier<Double> intakeSpeed;
    public final Supplier<Double> intakeReverseSpeed;
    public final Supplier<Double> rightStick;

    public final Trigger smartIntakeButton1;
    public final Trigger smartIntakeButton2;
    public final Button smartIndexerButton;
    public final Button highSmartShooterButton;
    public final Button lowSmartShooterButton;
    public final Button slowButton;
    public final Button overrideButton;
    public final Button indexerReverseButton;
    public final Button climberManualButton;
    public final Trigger intakeExtenderManualButton;
    public final Trigger intakeUpButton;
    public final Trigger intakeDownButton;

    /**
     * Constructs the Operator Interface.
     */
    public OI() {    
        xbox = new XboxController(Constants.OI.XBOX_PORT);

        teleopX = () -> -xbox.getLeftY();
        teleopTheta = () -> xbox.getLeftX();

        //intakeSpeed = () -> xbox.getLeftTriggerAxis();
        intakeReverseSpeed = () -> xbox.getRightTriggerAxis();
        rightStick = () -> -xbox.getRightY();

        smartIntakeButton1 = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
        smartIntakeButton2 = new Trigger(() -> xbox.getLeftTriggerAxis() > Constants.OI.DEADZONE);
        smartIndexerButton = new JoystickButton(xbox, XboxController.Button.kA.value);
        highSmartShooterButton = new JoystickButton(xbox, XboxController.Button.kX.value);
        lowSmartShooterButton = new JoystickButton(xbox, XboxController.Button.kY.value);
        slowButton = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
        overrideButton = new JoystickButton(xbox, XboxController.Button.kStart.value);
        indexerReverseButton = new JoystickButton(xbox, XboxController.Button.kB.value);
        climberManualButton = new JoystickButton(xbox, XboxController.Button.kBack.value);
        intakeExtenderManualButton = new Trigger(() -> xbox.getPOV() == 270);
        intakeUpButton = new Trigger(() -> xbox.getPOV() == 0);
        intakeDownButton = new Trigger(() -> xbox.getPOV() == 180);
    }
}