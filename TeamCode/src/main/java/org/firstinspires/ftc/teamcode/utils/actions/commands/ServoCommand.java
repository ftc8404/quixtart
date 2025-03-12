package org.firstinspires.ftc.teamcode.utils.actions.commands;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * A command to move a servo to a specified position.
 */
public class ServoCommand implements Command {
    /** The servo to move. */
    private final Servo servo;

    /** The position to move the servo to. */
    private final double position;

    public ServoCommand(Servo servo, double position) {
        this.servo = servo;
        this.position = position;
    }

    @Override
    public void run() {
        servo.setPosition(position);
    }
}
