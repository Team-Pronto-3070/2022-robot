package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Framework for the operator interface for the robot.
 * 
 * Contains necessary sensors and manual control objects (i.e. joysticks).
 * Assigns different functions of the robot to multiple inputs (i.e. bumper, button).
 */
public class OI {
    
    private Joystick joystick;
    public XboxController xbox;

    public final Supplier<Double> teleopX;
    public final Supplier<Double> teleopTheta;

    public final Supplier<Double> indexerForwardSpeed;
    public final Supplier<Double> indexerReverseSpeed;

    public final Button highShooterButton;
    public final Button lowShooterButton;
    public final Button smartIndexerButton;
    public final Button highSmartShooterButton;
    public final Button lowSmartShooterButton;
    public final Button slowButton;
    public final Button shooterOverrideButton;

    /**
     * Constructs the Operator Interface.
     */
    public OI() {    
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK:     
                joystick = new Joystick(Constants.OI.JOY_PORT);

                teleopX = () -> -joystick.getRawAxis(1);
                teleopTheta = () -> joystick.getRawAxis(0);

                indexerForwardSpeed = () -> joystick.getRawAxis(2);
                indexerReverseSpeed = () -> -joystick.getRawAxis(2);

                highShooterButton = new JoystickButton(joystick, 0);
                lowShooterButton = new JoystickButton(joystick, 1);
                smartIndexerButton = new JoystickButton(joystick, 3);
                highSmartShooterButton = new JoystickButton(joystick, 4);
                lowSmartShooterButton = new JoystickButton(joystick, 5);
                slowButton = new JoystickButton(joystick, 6);
                shooterOverrideButton = new JoystickButton(joystick, 7);

                break;

            default:
            case XBOX: 
                xbox = new XboxController(Constants.OI.XBOX_PORT);

                teleopX = () -> -xbox.getLeftY();
                teleopTheta = () -> xbox.getLeftX();

                indexerForwardSpeed = () -> xbox.getLeftTriggerAxis();
                indexerReverseSpeed = () -> xbox.getRightTriggerAxis();

                highShooterButton = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
                lowShooterButton = new JoystickButton(xbox, XboxController.Button.kB.value);
                smartIndexerButton = new JoystickButton(xbox, XboxController.Button.kA.value);
                highSmartShooterButton = new JoystickButton(xbox, XboxController.Button.kX.value);
                lowSmartShooterButton = new JoystickButton(xbox, XboxController.Button.kY.value);
                slowButton = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);
                shooterOverrideButton = new JoystickButton(xbox, XboxController.Button.kStart.value);

                break;

        }
    }
}