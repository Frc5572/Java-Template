package frc.lib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * A {@link Axis} treated as a {@link Trigger} that gets its state from a {@link GenericHID}.
 */
public class AxisButton extends Trigger {
    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     */
    public AxisButton(GenericHID joystick, int axisNumber) {
        super(() -> (Math.abs(joystick.getRawAxis(axisNumber)) > 0.4));
    }

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     * @param sensitivity The value the axis has to move to be considered as pressed
     */
    public AxisButton(GenericHID joystick, int axisNumber, double sensitivity) {
        super(() -> Math.abs(joystick.getRawAxis(axisNumber)) > sensitivity);
    }

}
