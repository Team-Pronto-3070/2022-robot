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
 */
public class OI {
    
    private Joystick joystick;
    private XboxController xbox;

    public final Button ClimberDownButton;
    public final Button ClimberUpButton;
    public final Button ClimberForwardButton;
    public final Button ClimberBackwardButton;
    public final Supplier<Double> ClimberSpeed;

    /**
     * Constructs the Operator Interface.
     */
    public OI() {    
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK:     
                joystick = new Joystick(Constants.OI.JOY_PORT);
                ClimberDownButton = new JoystickButton(xbox, 0);
                ClimberUpButton = new JoystickButton(xbox, 0);
                ClimberForwardButton = new JoystickButton(xbox, 0);
                ClimberBackwardButton = new JoystickButton(xbox, 0);
                ClimberSpeed = () -> joystick.getRawAxis(2);
                break;
            default:
            case XBOX: 
                xbox = new XboxController(Constants.OI.XBOX_PORT);
                ClimberDownButton = new JoystickButton(xbox, XboxController.Button.kA.value);
                ClimberUpButton = new JoystickButton(xbox, XboxController.Button.kX.value);
                ClimberForwardButton = new JoystickButton(xbox, XboxController.Button.kB.value);
                ClimberBackwardButton = new JoystickButton(xbox, XboxController.Button.kY.value);
                ClimberSpeed = () -> xbox.getRightTriggerAxis();
                break;

        }
    }

    public double getX(){
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK:
                return -joystick.getRawAxis(1);
            case XBOX: 
                return -xbox.getLeftY();
            default: 
                throw new RuntimeException("No controller selected");
        }
        //return -joystick.getRawAxis(1);
    }

    public double getTheta(){
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK: 
                return joystick.getRawAxis(0);
            case XBOX: 
                return xbox.getLeftX();
            default: 
                throw new RuntimeException("No controller selected");
        }
        //return joystick.getRawAxis(0);
    }
}