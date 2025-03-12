package org.firstinspires.ftc.teamcode.utils.actions.commands;

import org.firstinspires.ftc.teamcode.utils.Log;

/**
 * A command that waits for a specified amount of time.
 */
public class WaitCommand implements Command {
    /** The time to wait in milliseconds. */
    private final long milliseconds;

    public WaitCommand(long milliseconds) {
        this.milliseconds = milliseconds;
    }

    @Override
    public void run() {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Log.logError("WaitCommand", e,
                    "Interrupted while waiting for %d milliseconds", milliseconds);
        }
    }

    /** Gets the time to wait in milliseconds. */
    public long getMilliseconds() {
        return milliseconds;
    }
}
