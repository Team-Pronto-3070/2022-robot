package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Framework for the operator interface for the robot.
 * 
 * Contains necessary sensors and manual control objects (i.e. joysticks).
 */
public class OI {
    
    /* Class Variable Declaration */
    private HashMap<String, JoystickButton> buttons;
    
    private Joystick joystick;
    private XboxController xbox;

    /**
     * Constructs the Operator Interface.
     */
    public OI() {
        /* Class Variable Instantiation */
        buttons = new HashMap<>();
    
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK:     
                joystick = new Joystick(Constants.OI.JOY_PORT);
                break;
            case XBOX: 
                xbox = new XboxController(Constants.OI.XBOX_PORT);
                break;

        }
    }

    /**
     * Add a particular button to the operator interface.
     * Although these buttons could technically be accessed using the joystick object(s),
     * this implementation makes it easier for the developers to understand
     * the functionality of each button and allows for smoother implementation
     * of configureButtonBindings() in RobotContainer.
     * 
     * @param name of the button.
     * @param joystick that the button belongs to.
     * @param number assigned to the button by the controller in the Driver Station.
     */
    public void addButton(String name, int number) {
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK:
                buttons.put(name, new JoystickButton(joystick, number));
                break;
            case XBOX:
                buttons.put(name, new JoystickButton(xbox, number));
        }
        //buttons.put(name, new JoystickButton(joystick, number));
    }

    /**
     * @param name of a joystick button in the OI.
     * @return A desired JoystickButton object from the OI.
     */
    public JoystickButton getButton(String name) {
        return buttons.get(name);
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