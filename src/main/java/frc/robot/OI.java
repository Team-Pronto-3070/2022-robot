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

    public final Supplier<Double> teleopX;
    public final Supplier<Double> teleopTheta;

    public final Supplier<Double> IndexerSpeed;

    public final Button shooterButton;
    public final Button getDashboardShooterRPM;

    /**
     * Constructs the Operator Interface.
     */
    public OI() {    
        switch (Constants.OI.CONTROLLER) {
            case JOYSTICK:     
                joystick = new Joystick(Constants.OI.JOY_PORT);

                teleopX = () -> -joystick.getRawAxis(1);
                teleopTheta = () -> joystick.getRawAxis(0);

                IndexerSpeed = () -> joystick.getRawAxis(2);

                shooterButton = new JoystickButton(joystick, 0);
                getDashboardShooterRPM = new JoystickButton(joystick, 1);

                break;

            default:
            case XBOX: 
                xbox = new XboxController(Constants.OI.XBOX_PORT);

                teleopX = () -> -xbox.getLeftY();
                teleopTheta = () -> xbox.getLeftX();

                IndexerSpeed = () -> xbox.getLeftTriggerAxis();

                shooterButton = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
                getDashboardShooterRPM = new JoystickButton(xbox, XboxController.Button.kY.value);

                break;

        }
    }
}