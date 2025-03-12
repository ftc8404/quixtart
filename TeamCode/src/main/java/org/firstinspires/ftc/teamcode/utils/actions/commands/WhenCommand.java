package org.firstinspires.ftc.teamcode.utils.actions.commands;

import org.firstinspires.ftc.teamcode.utils.config.ServoConfig;

/**
 * A command that pauses execution of an action until a condition is met.
 * Has higher precedence than a wait command. When commands will always be run first if present
 * consecutively.
 */
public class WhenCommand implements Command {
    /** The servo to check. */
    public final ServoConfig servo;

    /** The position to check for. */
    public final double position;

    /** The tolerance for the position's actual reading. */
    public double tolerance = 0.03;

    /** The timeout for waiting for the condition to be met, in milliseconds. */
    public int timeout = 1500;

    /** The start of wait time, set by the scheduler. */
    public long startTime = 0;

    /** Whether or not the scheduler has set the start time. */
    public boolean startSet = false;

    public WhenCommand(ServoConfig servo, double position) {
        this.servo = servo;
        this.position = position;
    }

    public WhenCommand(ServoConfig servo, double position, double tolerance) {
        this.servo = servo;
        this.position = position;
        this.tolerance = tolerance;
    }

    public WhenCommand(ServoConfig servo, double position, int timeout) {
        this.servo = servo;
        this.position = position;
        this.timeout = timeout;
    }

    public WhenCommand(ServoConfig servo, double position, double tolerance, int timeout) {
        this.servo = servo;
        this.position = position;
        this.tolerance = tolerance;
        this.timeout = timeout;
    }

    @Override
    public void run() {}
}
