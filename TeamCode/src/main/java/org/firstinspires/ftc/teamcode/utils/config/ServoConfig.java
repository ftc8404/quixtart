package org.firstinspires.ftc.teamcode.utils.config;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Log;

import java.util.List;

/**
 * Encapsulates the servo hardware class and any saved positions for the servo.
 */
public abstract class ServoConfig {
    /** The servo hardware class. */
    public Servo servo;

    /** The servo's analog input. */
    public AnalogInput analogInput = null;

    /** If the analog input should be reversed. */
    public boolean reverseAnalogInput = false;

    /** Max voltage for the analog input. */
    public static final double MAX_VOLTAGE = 3.3;

    /**
     * Map for the analog reading to servo position, respectively.
     * Requires at least two points to be defined.
     */
    public List<double[]> analogToPosition;

    public ServoConfig() {
        this.servo = null;
    }

    public ServoConfig(Servo servo) {
        this.servo = servo;
    }

    public ServoConfig(Servo servo, AnalogInput analogInput) {
        this.servo = servo;
        this.analogInput = analogInput;
    }

    /** Gets the current position of the servo based on analog input or servo position. */
    public double getPosition() {
        if (analogInput != null) {
            double position = analogInput.getVoltage() / MAX_VOLTAGE;
            position = reverseAnalogInput ? 1 - position : position;

            // check if the map is defined
            if (analogToPosition != null && analogToPosition.size() > 1) {
                // obtain the two mapped points
                double[] point1 = analogToPosition.get(0);
                double[] point2 = analogToPosition.get(1);

                // interpolate between the two points
                double slope = (point2[1] - point1[1]) / (point2[0] - point1[0]);
                position = slope * (position - point1[0]) + point1[1];
            }

            return position;
        }
        return servo.getPosition();
    }
}
