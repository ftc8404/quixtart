package org.firstinspires.ftc.teamcode.utils.config;

/**
 * Configuration class for robot poses in Pedro so that it can be updated with FTC Dashboard.
 */
public class PoseConfig {
    /** The x-coordinate of the pose. */
    public double x;

    /** The y-coordinate of the pose. */
    public double y;

    /** The heading of the pose, in radians. */
    public double heading;

    public PoseConfig(double inX, double inY, double inHeading) {
        x = inX;
        y = inY;
        heading = inHeading;
    }

    /**
     * Get the x-coordinate of the pose.
     * @return The x-coordinate of the pose.
     */
    public double getX() {
        return x;
    }

    /**
     * Get the y-coordinate of the pose.
     * @return The y-coordinate of the pose.
     */
    public double getY() {
        return y;
    }

    /**
     * Get the heading of the pose, in radians.
     * @return The heading of the pose, in radians.
     */
    public double getHeading() {
        return heading;
    }
}
