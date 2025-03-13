package org.firstinspires.ftc.teamcode.utils.mpc;
import java.util.Arrays;

public class MecanumModel {
    /**
     * A mecanum wheel drivebase robot with 3 degrees of freedom: x, y, and rotation.
     */

    // Physical parameters (real system)
    private double mass;                  // Robot mass (kg)
    private double inertia;               // Moment of inertia (kg·m²)
    private double frictionX;             // Friction coefficient for x-direction
    private double frictionY;             // Friction coefficient for y-direction (typically higher than x)
    private double frictionTheta;         // Rotational friction coefficient
    private double maxForceX;             // Maximum forward/backward force
    private double maxForceY;             // Maximum left/right force
    private double maxTorque;             // Maximum rotational torque

    // State: [x, y, theta, vx, vy, omega]
    // x, y: position; theta: orientation; vx, vy: linear velocities in world frame; omega: angular velocity
    private double[] state;

    public MecanumModel(double mass, double inertia, double frictionX, double frictionY, double frictionTheta,
                        double maxForceX, double maxForceY, double maxTorque) {
        // Physical parameters (real system)
        this.mass = mass;
        this.inertia = inertia;
        this.frictionX = frictionX;
        this.frictionY = frictionY;
        this.frictionTheta = frictionTheta;
        this.maxForceX = maxForceX;
        this.maxForceY = maxForceY;
        this.maxTorque = maxTorque;

        // Initialize state
        this.state = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    /**
     * Default constructor with predefined parameters
     */
    public MecanumModel() {
        this(17.4, .51124, 0.8, 1.0, 0.5, 0.5, 1.0, 1.0);
    }

    /**
     * Advance the system one step with input u = [fx, fy, torque] and deltaTime
     */
    public double[] step(double[] u, double deltaTime) {
        this.state = this.nextState(this.state, u, deltaTime);
        return Arrays.copyOf(this.state, this.state.length);
    }

    /**
     * Advance the system one step using the solver method
     */
    public double[] stepSolver(double[] u, double deltaTime) {
        this.state = this.nextStateSolver(this.state, u, deltaTime);
        return Arrays.copyOf(this.state, this.state.length);
    }

    /**
     * Rotate a 2D vector by theta radians
     */
    private double[] rotateVector(double[] v, double theta) {
        double cosTheta = Math.cos(theta);
        double sinTheta = Math.sin(theta);
        return new double[]{
                v[0] * cosTheta - v[1] * sinTheta,
                v[0] * sinTheta + v[1] * cosTheta
        };
    }

    /**
     * Calculate next state using system parameters and deltaTime
     */
    public double[] nextState(double[] state, double[] u, double deltaTime) {
        double x = state[0], y = state[1], theta = state[2];
        double vx = state[3], vy = state[4], omega = state[5];
        double fxRobot = u[0], fyRobot = u[1], torque = u[2];

        // Apply force limits
        fxRobot = clip(fxRobot, -this.maxForceX, this.maxForceX);
        fyRobot = clip(fyRobot, -this.maxForceY, this.maxForceY);
        torque = clip(torque, -this.maxTorque, this.maxTorque);

        // Convert robot-frame forces to world-frame forces
        double[] worldForces = rotateVector(new double[]{fxRobot, fyRobot}, theta);
        double fx = worldForces[0];
        double fy = worldForces[1];

        // Convert world-frame velocities to robot-frame for applying friction
        double[] robotVelocities = rotateVector(new double[]{vx, vy}, -theta);
        double vxRobot = robotVelocities[0];
        double vyRobot = robotVelocities[1];

        // Apply friction in robot frame
        double frictionX = this.frictionX * vxRobot;
        double frictionY = this.frictionY * vyRobot;

        // Rotate friction forces back to world frame
        double[] worldFriction = rotateVector(new double[]{frictionX, frictionY}, theta);

        // Accelerations in world frame with friction
        double ax = (fx - worldFriction[0]) / this.mass;
        double ay = (fy - worldFriction[1]) / this.mass;
        double alpha = (torque - this.frictionTheta * omega) / this.inertia;

        // Update velocities with deltaTime
        double vxNew = vx + ax * deltaTime;
        double vyNew = vy + ay * deltaTime;
        double omegaNew = omega + alpha * deltaTime;

        // Update positions (integrate velocities) with deltaTime
        // For a more accurate simulation, we use the average of old and new velocities
        double xNew = x + ((vxNew + vx) / 2) * deltaTime;
        double yNew = y + ((vyNew + vy) / 2) * deltaTime;
        double thetaNew = theta + ((omegaNew + omega) / 2) * deltaTime;

        // Normalize theta to [-pi, pi)
        thetaNew = ((thetaNew + Math.PI) % (2 * Math.PI)) - Math.PI;

        return new double[]{xNew, yNew, thetaNew, vxNew, vyNew, omegaNew};
    }

    /**
     * Calculate next state using a system of equations approach
     * This provides a more accurate solution by solving the differential equations
     * rather than using the step-by-step velocity and position updates
     */
    public double[] nextStateSolver(double[] state, double[] u, double deltaTime) {
        double x = state[0], y = state[1], theta = state[2];
        double vx = state[3], vy = state[4], omega = state[5];
        double fxRobot = u[0], fyRobot = u[1], torque = u[2];

        // Apply force limits
        fxRobot = clip(fxRobot, -this.maxForceX, this.maxForceX);
        fyRobot = clip(fyRobot, -this.maxForceY, this.maxForceY);
        torque = clip(torque, -this.maxTorque, this.maxTorque);

        // Convert robot-frame forces to world-frame forces
        double[] worldForces = rotateVector(new double[]{fxRobot, fyRobot}, theta);
        double fx = worldForces[0];
        double fy = worldForces[1];

        // Set up the system of equations
        // For translational motion: m*(dv/dt) + c*v = F
        // For rotational motion: I*(dω/dt) + c_θ*ω = τ

        // Calculate robot-frame velocities for friction
        double[] robotVelocities = rotateVector(new double[]{vx, vy}, -theta);
        double vxRobot = robotVelocities[0];
        double vyRobot = robotVelocities[1];

        // Damping coefficients
        double cx = this.frictionX;
        double cy = this.frictionY;
        double cTheta = this.frictionTheta;

        // Solve the differential equations analytically
        // For equation of form: dv/dt + (c/m)*v = F/m
        // Solution is: v(t) = v(0)*e^(-(c/m)*t) + (F/c)*(1 - e^(-(c/m)*t))

        // Time constants
        double tauX = this.mass / cx;
        double tauY = this.mass / cy;
        double tauTheta = this.inertia / cTheta;

        // Exponential decay factors
        double expFactorX = Math.exp(-deltaTime / tauX);
        double expFactorY = Math.exp(-deltaTime / tauY);
        double expFactorTheta = Math.exp(-deltaTime / tauTheta);

        // Calculate steady-state velocities if forces remained constant
        double vxRobotSS = fxRobot / cx;
        double vyRobotSS = fyRobot / cy;
        double omegaSS = torque / cTheta;

        // New robot-frame velocities
        double vxRobotNew = vxRobot * expFactorX + vxRobotSS * (1 - expFactorX);
        double vyRobotNew = vyRobot * expFactorY + vyRobotSS * (1 - expFactorY);
        double omegaNew = omega * expFactorTheta + omegaSS * (1 - expFactorTheta);

        // Convert back to world frame
        double[] worldVelocitiesNew = rotateVector(new double[]{vxRobotNew, vyRobotNew}, theta);
        double vxNew = worldVelocitiesNew[0];
        double vyNew = worldVelocitiesNew[1];

        // For position, integrate the velocity over time
        // For constant acceleration: p(t) = p(0) + v(0)*t + 0.5*a*t^2
        // For our exponential velocity profile:
        // p(t) = p(0) + v(0)*tau*(1-e^(-t/tau)) + (F/c)*t - (F/c)*tau*(1-e^(-t/tau))

        // Average velocities over the time interval for position update
        // This is a simplification - a more accurate solution would integrate the exact velocity profiles
        double vxAvg = (vx + vxNew) / 2;
        double vyAvg = (vy + vyNew) / 2;
        double omegaAvg = (omega + omegaNew) / 2;

        double xNew = x + vxAvg * deltaTime;
        double yNew = y + vyAvg * deltaTime;
        double thetaNew = theta + omegaAvg * deltaTime;

        // Normalize theta to [-pi, pi)
        thetaNew = ((thetaNew + Math.PI) % (2 * Math.PI)) - Math.PI;

        return new double[]{xNew, yNew, thetaNew, vxNew, vyNew, omegaNew};
    }

    /**
     * Helper method to clip a value between min and max
     */
    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    /**
     * Get the current state
     */
    public double[] getState() {
        return Arrays.copyOf(this.state, this.state.length);
    }

    /**
     * Set the current state
     */
    public void setState(double[] state) {
        if (state.length != 6) {
            throw new IllegalArgumentException("State must have 6 elements");
        }
        this.state = Arrays.copyOf(state, state.length);
    }

    public double getMaxForceX() {
        return maxForceX;
    }

    public double getMaxForceY() {
        return maxForceY;
    }

    public double getMaxTorque() {
        return maxTorque;
    }
}