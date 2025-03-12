package org.firstinspires.ftc.teamcode.utils.pid;

/**
 * An interface for a PID controller.
 * All PID controllers should implement this interface.
 */
public interface PIDController {
    /**
     * Sets the constants for the PID controller.
     */
    public void setConstants();

    /**
     * Updates the PID controller.
     * @return The output power of the PID controller.
     */
    public double update();

    /**
     * Checks if the PID controller is at the target position.
     * @return Whether the PID controller is at the target position.
     */
    public boolean isAtTargetPosition();

    /**
     * Gets the current position of the PID controller.
     * @return The current position of the PID controller.
     */
    public double getTargetPosition();

    /**
     * Sets the target position of the PID controller.
     * @param targetPosition The target position of the PID controller.
     */
    public void setTargetPosition(double targetPosition);
}
