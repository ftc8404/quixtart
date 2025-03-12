package org.firstinspires.ftc.teamcode.utils.actions;

import org.firstinspires.ftc.teamcode.utils.actions.commands.Command;
import org.firstinspires.ftc.teamcode.utils.actions.commands.WhenCommand;
import org.firstinspires.ftc.teamcode.utils.config.ServoConfig;

import java.util.ArrayList;
import java.util.List;

/**
 * A class for building actions to be executed by the robot.
 */
public abstract class ActionBuilder {
    /** The list of commands to be executed. */
    public final List<Command> commands;

    public ActionBuilder(List<Command> commands) {
        this.commands = commands;
    }

    /** Builds the commands into an action. */
    public Action build() {
        ArrayList<Command> copyCommands = new ArrayList<>(commands);
        Action action = new Action(copyCommands);
        commands.clear();
        return action;
    }

    public abstract ActionBuilder pause(long milliseconds);
}
