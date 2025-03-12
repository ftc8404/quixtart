package org.firstinspires.ftc.teamcode.utils.actions;

import org.firstinspires.ftc.teamcode.utils.actions.commands.Command;
import org.firstinspires.ftc.teamcode.utils.actions.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.utils.actions.commands.WhenCommand;
import org.firstinspires.ftc.teamcode.utils.config.ServoConfig;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

/**
 * A series of commands to be run by the robot.
 */
public class Action implements Runnable {
    /** The list of commands to run. */
    private final List<Command> commands;

    /** The time delay for the commands. */
    private long ms = 0;

    /** A timer for timer tasks. */
    private final Timer timer = new Timer();

    /** A pointer to the current command. */
    private int currentCommand = 0;

    /** Whether the action is waiting for a condition to be met. */
    private boolean waiting = false;

    public Action(List<Command> commands) {
        this.commands = commands;
    }

    @Override
    public void run() {
        // assume waiting condition is met when run again and move on to next command
        if (waiting) {
            currentCommand++;
            waiting = false;
        }

        // iterate through each of the commands, keeping track of the state
        while (currentCommand < commands.size()) {
            Command command = commands.get(currentCommand);
            if (command instanceof WaitCommand) {
                ms += ((WaitCommand) command).getMilliseconds();
            } else if (command instanceof WhenCommand) {
                waiting = true;
                ms = 0;
                break;
            } else if (ms == 0) {
                command.run();
            } else {
                TimerTask task = new TimerTask() {
                    @Override
                    public void run() {
                        command.run();
                    }
                };
                timer.schedule(task, ms);
            }
            currentCommand++;
        }

        // if all commands are run, reset the state of the action
        if (currentCommand == commands.size()) {
            currentCommand = 0;
            ms = 0;
            waiting = false;
        }
    }

    /** Get if the action is waiting for a condition to be met. */
    public boolean isWaiting() {
        return waiting;
    }

    /** Gets the command. */
    public Command getCurrentCommand() {
        return commands.get(currentCommand);
    }

    /**
     * Uses analog input to determine when to run the next action.
     * @param servo the servo to check position for.
     * @param position the position to check for.
     */
    public Action when(ServoConfig servo, double position) {
        WhenCommand command = new WhenCommand(servo, position);
        commands.add(command);
        return this;
    }

    /**
     * Uses analog input to determine when to run the next action.
     * @param servo the servo to check position for.
     * @param position the position to check for.
     * @param tolerance the tolerance for checking the position
     */
    public Action when(ServoConfig servo, double position, double tolerance) {
        WhenCommand command = new WhenCommand(servo, position, tolerance);
        commands.add(command);
        return this;
    }

    /**
     * Adds a wait command to the action to pause.
     * @param milliseconds the time to wait in milliseconds.
     * @return the action with the wait command added.
     */
    public Action pause(long milliseconds) {
        commands.add(new WaitCommand(milliseconds));
        return this;
    }

    /**
     * Appends the commands from another action to this action, effectively combining them.
     * @param action The action to append.
     * @return This action.
     */
    public Action append(Action action) {
        commands.addAll(action.commands);
        return this;
    }
}
