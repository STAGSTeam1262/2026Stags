package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends CommandXboxController {
    /***
     * Class that adds additional controls for things like vibration and commands for sequencing rumble more easily.
     * 
     * @param id id of the controller being handled, which will create a new controller object, accessed through getController().
     */
    public Controller(int id) {
        super(id);
    }

    /**
     * 
     * @return If the controller is connected to the Driver Station.
     */
    public boolean isConnected() {
        return isConnected();
    }

    /*** Command to make the controller vibrate with a certain power and position.
     * 
     * @param type Rumble on either the left or right vibration motor, or both.
     * @param power The power of the vibration.
     */
    public Command rumble(RumbleType type, double power) {
        return Commands.runOnce(() -> setRumble(type, power));
    }

    /***
     * Stops rumble on both vibration motors of the controller.
     */
    public void cancelRumble() {
        setRumble(RumbleType.kBothRumble, 0);
    }

    /***
     * Command to stop rumble on both vibration motors of the controller.
     */
    public Command stopRumble() {
        return Commands.runOnce(() -> cancelRumble());
    }

    /** Command to make the controller rumble on certain vibration motors for a set period of time, and then stops the vibration.
     * 
     * @param type Rumble on either the left or right vibration motor, or both.
     * @param power Power of the vibration.
     * @param time Time in seconds that the vibration lasts.
     * @return Command to make controller rumble.
     */
    public Command rumbleThenStop(RumbleType type, double power, double time) {
        return rumble(type, power).andThen(new WaitCommand(time)).andThen(stopRumble()).andThen(new WaitCommand(time));
    }

    /** Command to make the controller rumble on certain vibration motors for a set period of time, and then stops the vibration. This sequence repeats twice after.
     * 
     * @param type Rumble on either the left or right vibration motor, or both.
     * @param power Power of each vibration.
     * @param time Time in seconds that each vibration lasts.
     * @return Command to make controller rumble.
     */
    public Command blinkRumble(double time, RumbleType type, double power) {
        return rumbleThenStop(type, power, time)
            .andThen(rumbleThenStop(type, power, time))
            .andThen(rumbleThenStop(type, power, time));
    }
    
}
