package org.firstinspires.ftc.teamcode.utils.actions.commands;

import org.firstinspires.ftc.teamcode.hw.MecanumDrive;

import java.util.Timer;
import java.util.TimerTask;

public class DriveCommand implements Command {
    /** The drivebase class. */
    private final MecanumDrive drive;

    /** The power change in the x direction. */
    private final double x;

    /** The power change in the y direction. */
    private final double y;

    /** The power change in rotation. */
    private final double rot;

    /** The movement time in milliseconds. */
    private final long time;

    /** Timer for timer tasks. */
    private final Timer timer = new Timer();

    /**
     * Constructor for a new drive command.
     * @param drive the drive motor class
     * @param x The desired movement along the X-axis (-1.0 to 1.0)
     *          Positive values indicate forward movement
     *          Negative values indicate backward movement
     * @param y The desired movement along the Y-axis (-1.0 to 1.0)
     *          Positive values indicate movement to the right
     *          Negative values indicate movement to the left
     * @param rot The desired rotation (-1.0 to 1.0)
     *           Positive values indicate clockwise rotation
     *           Negative values indicate counter-clockwise rotation
     * @param time The time for it to move in milliseconds
     */
    public DriveCommand(MecanumDrive drive, double x, double y, double rot, long time) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.rot = rot;
        this.time = time;
    }

    /** Sets zero power to motors for it to brake. */
    private void zero() {
        double[] powers = drive.calculatePowers(0, 0, 0);
        drive.setPowers(powers);
    }

    @Override
    public void run() {
        // timer task for stopping the robot
        TimerTask stop = new TimerTask() {
            @Override
            public void run() {
                zero();
            }
        };

        // set powers to the drivebase
        double[] powers = drive.calculatePowers(x, y, rot);
        drive.setPowers(powers);
        timer.schedule(stop, time);
    }
}
