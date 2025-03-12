package org.firstinspires.ftc.teamcode.utils;

import static com.sun.tools.javac.util.Assert.error;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * A utility class for logging messages to the robot logs.
 */
public class Log {
    /**
     * Logs an error message to the robot logs.
     *
     * @param tag The tag to use for the log message.
     * @param e The exception to log.
     * @param format The format string for the log message.
     * @param args The arguments to the format string.
     */
    public static void logError(String tag, Exception e, String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.ee(tag, e, message);
        error(message);
    }

    /**
     * Logs an informational message to the robot logs.
     *
     * @param tag The tag to use for the log message.
     * @param format The format string for the log message.
     * @param args The arguments to the format string.
     */
    public static void logInfo(String tag, String format, Object... args) {
        String message = String.format(format, args);
        RobotLog.ii(tag, message);
    }
}
