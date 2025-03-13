package org.firstinspires.ftc.teamcode.utils;

/**
 * A class to implement a rising edge detector for a button.
 */
public class RisingEdge {
    /** Indicates whether the button was previously pressed. */
    private boolean down;

    /**
     * Constructs a RisingEdge object with the initial state set to not pressed.
     */
    public RisingEdge() {
        down = false;
    }

    /**
     * Updates the state of the button and detects a rising edge.
     * @param input The current state of the button (true if pressed, false otherwise).
     * @return true if a rising edge is detected (not pressed before but is now), false otherwise.
     */
    public boolean update(boolean input) {
        if (input) {
            if (!down) {
                down = true;
                return true;
            }
        } else {
            down = false;
        }
        return false;
    }
}
