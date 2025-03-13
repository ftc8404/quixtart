package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RisingEdge;
import org.firstinspires.ftc.teamcode.utils.actions.Action;

import java.util.HashMap;
import java.util.Map;

/**
 * The Controllers class manages the mapping of gamepad buttons to actions.
 */
public class Controllers {
    /**
     * The map of buttons to actions.
     */
    private final Map<Button, Action> actionMap;
    /**
     * The map of buttons to their states as a rising edge detector.
     */
    private final Map<Button, RisingEdge> buttonMap;
    /**
     * The gamepad object for the first driver.
     */
    private final Gamepad gamepad1;
    /**
     * The gamepad object for the second driver.
     */
    private final Gamepad gamepad2;

    /**
     * Constructs a Controllers object with the specified gamepads.
     *
     * @param gamepad1 The gamepad object for the first driver.
     * @param gamepad2 The gamepad object for the second driver.
     */
    public Controllers(Gamepad gamepad1, Gamepad gamepad2) {
        // initialize an empty button to action map
        actionMap = new HashMap<>();

        // set buttons to a new rising edge detector for each button
        buttonMap = new HashMap<>();
        for (Button button : Button.values()) {
            buttonMap.put(button, new RisingEdge());
        }

        // set the gamepad objects
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Binds a button to an action.
     *
     * @param button The button to bind.
     * @param action The action to bind to the button.
     */
    public void bind(Button button, Action action) {
        actionMap.put(button, action);
    }

    /**
     * Updates the state of the buttons and executes the corresponding actions if a button is pressed.
     */
    public void update() {
        for (Map.Entry<Button, Action> entry : actionMap.entrySet()) {
            Button button = entry.getKey();
            Action action = entry.getValue();
            RisingEdge buttonState = buttonMap.get(button);
            if (buttonState != null && buttonState.update(isPressed(button))) {
                action.run();
            }
        }
    }

    /**
     * Checks if a button is pressed.
     *
     * @param button The button to check.
     * @return true if the button is pressed, false otherwise.
     */
    public boolean isPressed(Button button) {
        switch (button) {
            case CIRCLE_1:
                return gamepad1.circle;
            case CIRCLE_2:
                return gamepad2.circle;
            case SQUARE_1:
                return gamepad1.square;
            case SQUARE_2:
                return gamepad2.square;
            case TRIANGLE_1:
                return gamepad1.triangle;
            case TRIANGLE_2:
                return gamepad2.triangle;
            case CROSS_1:
                return gamepad1.cross;
            case CROSS_2:
                return gamepad2.cross;
            case LEFT_BUMPER_1:
                return gamepad1.left_bumper;
            case LEFT_BUMPER_2:
                return gamepad2.left_bumper;
            case RIGHT_BUMPER_1:
                return gamepad1.right_bumper;
            case RIGHT_BUMPER_2:
                return gamepad2.right_bumper;
            case LEFT_TRIGGER_1:
                return gamepad1.left_trigger > 0.5;
            case LEFT_TRIGGER_2:
                return gamepad2.left_trigger > 0.5;
            case RIGHT_TRIGGER_1:
                return gamepad1.right_trigger > 0.5;
            case RIGHT_TRIGGER_2:
                return gamepad2.right_trigger > 0.5;
            case DPAD_UP_1:
                return gamepad1.dpad_up;
            case DPAD_UP_2:
                return gamepad2.dpad_up;
            case DPAD_DOWN_1:
                return gamepad1.dpad_down;
            case DPAD_DOWN_2:
                return gamepad2.dpad_down;
            case DPAD_LEFT_1:
                return gamepad1.dpad_left;
            case DPAD_LEFT_2:
                return gamepad2.dpad_left;
            case DPAD_RIGHT_1:
                return gamepad1.dpad_right;
            case DPAD_RIGHT_2:
                return gamepad2.dpad_right;
            default:
                return false;
        }
    }

    /**
     * The buttons on the gamepad.
     */
    public enum Button {
        CIRCLE_1,
        CIRCLE_2,
        SQUARE_1,
        SQUARE_2,
        TRIANGLE_1,
        TRIANGLE_2,
        CROSS_1,
        CROSS_2,
        LEFT_BUMPER_1,
        LEFT_BUMPER_2,
        RIGHT_BUMPER_1,
        RIGHT_BUMPER_2,
        LEFT_TRIGGER_1,
        LEFT_TRIGGER_2,
        RIGHT_TRIGGER_1,
        RIGHT_TRIGGER_2,
        DPAD_UP_1,
        DPAD_UP_2,
        DPAD_DOWN_1,
        DPAD_DOWN_2,
        DPAD_LEFT_1,
        DPAD_LEFT_2,
        DPAD_RIGHT_1,
        DPAD_RIGHT_2,
    }
}
