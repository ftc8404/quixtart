package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.utils.mpc.MecanumModel;

import java.util.Arrays;

@TeleOp
public class MPCTest extends OpMode {

    MecanumModel model, solverModel;
    double oldTime = 0, currentTime, deltaTime;
    double[] control = new double[3];
    double[] state = new double[6], solverState = new double[6];

    @Override
    public void init() {
        model = new MecanumModel();
        solverModel = new MecanumModel();
    }

    @Override
    public void loop() {
        if (oldTime == 0) {
            deltaTime = 0;
            currentTime = (double) System.currentTimeMillis() / 1000;
            oldTime = currentTime;
        } else {
            currentTime = (double) System.currentTimeMillis() / 1000;
            deltaTime = currentTime - oldTime;
            oldTime = currentTime;
        }

        telemetry.addData(Arrays.toString(control), deltaTime);
        solverModel.stepSolver(control, deltaTime);
        model.step(control, deltaTime);
        state = model.getState();
        solverState = solverModel.getState();

        plotSquareOnDashboard(state[0], state[1], Math.toDegrees(state[2]), 16.0, "red", 2);
        plotSquareOnDashboard(solverState[0], solverState[1], Math.toDegrees(solverState[2]), 16.0, "blue", 2);

        control[0] = gamepad1.left_stick_x * model.getMaxForceX();
        control[1] = -gamepad1.left_stick_y * model.getMaxForceY();
        control[2] = -gamepad1.right_stick_x * model.getMaxTorque();
    }

    public void plotSquareOnDashboard(double xCenter, double yCenter, double rotation,
                                      double sideLength, String color, int strokeWidth) {
        // Create telemetry packet for dashboard
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // Calculate half side length for corner calculations
        double halfSide = sideLength / 2.0;

        // Convert rotation to radians
        double rotationRad = Math.toRadians(rotation);

        // Calculate corner coordinates (relative to center before rotation)
        double[][] corners = {
                {-halfSide, -halfSide}, // Bottom left
                {halfSide, -halfSide},  // Bottom right
                {halfSide, halfSide},   // Top right
                {-halfSide, halfSide}   // Top left
        };

        // Apply rotation and translation to each corner
        double[][] rotatedCorners = new double[4][2];
        for (int i = 0; i < 4; i++) {
            // Rotation
            rotatedCorners[i][0] = corners[i][0] * Math.cos(rotationRad) - corners[i][1] * Math.sin(rotationRad);
            rotatedCorners[i][1] = corners[i][0] * Math.sin(rotationRad) + corners[i][1] * Math.cos(rotationRad);

            // Translation
            rotatedCorners[i][0] += xCenter;
            rotatedCorners[i][1] += yCenter;
        }

        // Set stroke properties
        canvas.setStroke(color);
        canvas.setStrokeWidth(strokeWidth);

        // Draw the square (connect the corners with lines)
        for (int i = 0; i < 4; i++) {
            int nextI = (i + 1) % 4;
            canvas.strokeLine(
                    rotatedCorners[i][0], rotatedCorners[i][1],
                    rotatedCorners[nextI][0], rotatedCorners[nextI][1]
            );
        }
        canvas.strokeLine(
                (rotatedCorners[0][0] + rotatedCorners[1][0] + rotatedCorners[2][0] + rotatedCorners[3][0]) / 4,
                (rotatedCorners[0][1] + rotatedCorners[1][1] + rotatedCorners[2][1] + rotatedCorners[3][1]) / 4,
                (rotatedCorners[2][0] + rotatedCorners[3][0]) / 2, (rotatedCorners[2][1] + rotatedCorners[3][1]) / 2
        );

        // Send packet to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
