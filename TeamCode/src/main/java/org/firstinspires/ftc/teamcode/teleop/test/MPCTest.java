package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.utils.mpc.MecanumModel;

@TeleOp
public class MPCTest extends OpMode {

    MecanumModel model;
    double oldTime = 0, currentTime, deltaTime;
    double[] control = new double[3];
    double[] state = new double[6];

    @Override
    public void init() {
        model = new MecanumModel();
    }

    @Override
    public void loop() {
        if (oldTime == 0) {
            deltaTime = 0;
        } else {
            currentTime = (double) System.nanoTime() / 1000000;
            deltaTime = currentTime - oldTime;
            currentTime = oldTime;
        }

        model.stepSolver(control, deltaTime);
        state = model.getState();

        plotSquareOnDashboard(state[0], state[1], Math.toDegrees(state[2]), 4.0, "red", 2);

        control[0] = gamepad1.left_stick_x;
        control[1] = -gamepad1.left_stick_y;
        control[2] = gamepad1.right_stick_x;
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

        // Send packet to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
