package org.firstinspires.ftc.teamcode.hw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.actions.Action;
import org.firstinspires.ftc.teamcode.utils.actions.commands.Command;
import org.firstinspires.ftc.teamcode.utils.actions.commands.DriveCommand;

import java.util.ArrayList;
import java.util.List;

public class MecanumDrive {
    /** The drive motors: 0 is front left, 1 is front right, 2 is back left, 3 is back right */
    private final DcMotor[] motors = new DcMotor[4];

    /** Motor directions for mecanum drive */
    private final int[] rev = { 1, 1, 1, 1 };

    /** Weight constant for longitudinal (forward/backward) movement */
    public static double VX_WEIGHT = 1;

    /** Weight constant for lateral (left/right) movement */
    public static double VY_WEIGHT = 1;

    /** Weight constant for rotational (clockwise/counter-clockwise) movement */
    public static double OMEGA_WEIGHT = 1.2;

    public void init(HardwareMap hwMap) {
        // get motors from hardware map
        motors[0] = hwMap.dcMotor.get("leftFront");
        motors[1] = hwMap.dcMotor.get("rightFront");
        motors[2] = hwMap.dcMotor.get("leftBack");
        motors[3] = hwMap.dcMotor.get("rightBack");

        // set motor directions
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD);

        // set zero power behavior
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     * Calculates the power distribution for the mecanum drivebase.
     * This method takes desired movement vectors and rotation, normalizes them if necessary,
     * and converts them into individual motor power values.
     * <p>
     * The resulting powers array contains values for each wheel in the following order:
     * - powers[0]: Front Left Motor
     * - powers[1]: Front Right Motor
     * - powers[2]: Back Left Motor
     * - powers[3]: Back Right Motor
     * <p>
     * The method uses weight constants (VX_WEIGHT, VY_WEIGHT, OMEGA_WEIGHT) to adjust
     * the relative importance of different movement components during normalization.
     *
     * @param x The desired movement along the X-axis (-1.0 to 1.0)
     *          Positive values indicate forward movement
     *          Negative values indicate backward movement
     * @param y The desired movement along the Y-axis (-1.0 to 1.0)
     *          Positive values indicate movement to the right
     *          Negative values indicate movement to the left
     * @param rot The desired rotation (-1.0 to 1.0)
     *           Positive values indicate clockwise rotation
     *           Negative values indicate counter-clockwise rotation
     * @return An array of length 4 containing the calculated power values for each motor,
     *         with values normalized between -1.0 and 1.0
     */
    public double[] calculatePowers(double x, double y, double rot) {
        // initialize array to store power values for each motor
        double[] powers = new double[4];

        // if sum of absolute movement components exceed 1, normalize them
        if (Math.abs(x) + Math.abs(y) + Math.abs(rot) > 1) {
            // calculate the weighted sum of absolute movement components
            double denom = VX_WEIGHT * Math.abs(x) + VY_WEIGHT * Math.abs(y) + OMEGA_WEIGHT * Math.abs(rot);

            x *= VX_WEIGHT / denom;
            y *= VY_WEIGHT / denom;
            rot *= OMEGA_WEIGHT / denom;
        }

        // calculate power for each motor using mecanum drive equations
        powers[0] = -x + y + rot;
        powers[1] = x + y - rot;
        powers[2] = x + y + rot;
        powers[3] = -x + y - rot;

        return powers;
    }

    public Action move(double x, double y, double rot, long time) {
        List<Command> moving = new ArrayList<>();
        DriveCommand drive = new DriveCommand(this, x, y, rot, time);
        moving.add(drive);
        return new Action(moving);
    }

    /**
     * Sets the power values for each motor in the mecanum drivebase.
     *
     * @param powers An array of length 4 containing the power values for each motor,
     * - powers[0]: Front Left Motor
     * - powers[1]: Front Right Motor
     * - powers[2]: Back Left Motor
     * - powers[3]: Back Right Motor
     */
    public void setPowers(double[] powers) {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(rev[i] * powers[i]);
        }
    }
}
