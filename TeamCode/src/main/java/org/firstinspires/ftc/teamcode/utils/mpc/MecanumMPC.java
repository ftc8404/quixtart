package org.firstinspires.ftc.teamcode.utils.mpc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.SimpleValueChecker;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.CMAESOptimizer;
import org.apache.commons.math3.random.JDKRandomGenerator;

public class MecanumMPC {
    /**
     * MPC controller for the mecanum robot.
     */
    private MecanumModel system;
    private int horizon;       // Prediction horizon
    private double qPos;       // Position cost weight
    private double qTheta;     // Orientation cost weight
    private double r;          // Control cost weight
    private double deltaTime;  // Time step for prediction

    // Control constraints based on model limitations
    private double maxForceX;  // Maximum X force
    private double maxForceY;  // Maximum Y force
    private double maxTorque;  // Maximum torque

    // Reference trajectory function
    private TrajectoryGenerator trajectoryGenerator;

    /**
     * Functional interface for generating reference trajectories
     */
    public interface TrajectoryGenerator {
        double[] getReference(double time);
    }

    /**
     * Constructor for MPC controller with specified parameters
     */
    public MecanumMPC(MecanumModel system, int horizon, double qPos, double qTheta, double r, double deltaTime) {
        this.system = system;
        this.horizon = horizon;
        this.qPos = qPos;
        this.qTheta = qTheta;
        this.r = r;
        this.deltaTime = deltaTime;

        // Get constraints from the model if possible, otherwise use defaults
        try {
            this.maxForceX = system.getMaxForceX();
            this.maxForceY = system.getMaxForceY();
            this.maxTorque = system.getMaxTorque();
        } catch (Exception e) {
            // If getters don't exist, use reasonable defaults
            this.maxForceX = 0.5;
            this.maxForceY = 0.5;
            this.maxTorque = 1.0;
        }

        // Default trajectory generator (circular path)
        this.trajectoryGenerator = new TrajectoryGenerator() {
            @Override
            public double[] getReference(double time) {
                double radius = 5.0;
                double angularVelocity = 0.1;
                double angle = angularVelocity * time;

                return new double[]{
                        radius * Math.cos(angle),  // x position
                        radius * Math.sin(angle),  // y position
                        angle,                     // orientation (theta)
                        -radius * angularVelocity * Math.sin(angle),  // vx
                        radius * angularVelocity * Math.cos(angle),   // vy
                        angularVelocity                               // omega
                };
            }
        };
    }

    /**
     * Constructor with default values
     */
    public MecanumMPC(MecanumModel system) {
        this(system, 5, 1.0, 2.0, 0.1, 0.1);
    }

    /**
     * Set a custom trajectory generator
     */
    public void setTrajectoryGenerator(TrajectoryGenerator generator) {
        this.trajectoryGenerator = generator;
    }

    /**
     * Set control constraints manually
     */
    public void setControlConstraints(double maxForceX, double maxForceY, double maxTorque) {
        this.maxForceX = maxForceX;
        this.maxForceY = maxForceY;
        this.maxTorque = maxTorque;
    }

    /**
     * Set cost weights
     */
    public void setCostWeights(double qPos, double qTheta, double r) {
        this.qPos = qPos;
        this.qTheta = qTheta;
        this.r = r;
    }

    /**
     * Set prediction horizon
     */
    public void setHorizon(int horizon) {
        this.horizon = horizon;
    }

    /**
     * Set time step
     */
    public void setDeltaTime(double deltaTime) {
        this.deltaTime = deltaTime;
    }

    /**
     * Cost function for optimization.
     *
     * @param uSequence Flattened control sequence
     * @param currentState Current state of the robot
     * @param currentTime Current time value
     * @return Cost value
     */
    public double objective(double[] uSequence, double[] currentState, double currentTime) {
        double cost = 0;
        double[] state = Arrays.copyOf(currentState, currentState.length);
        double time = currentTime;

        // Reshape the flattened control sequence to 3D vectors
        double[][] uSequenceReshaped = new double[uSequence.length / 3][3];
        for (int i = 0; i < uSequence.length / 3; i++) {
            for (int j = 0; j < 3; j++) {
                uSequenceReshaped[i][j] = uSequence[i * 3 + j];
            }
        }

        // Sum costs over the prediction horizon
        for (int i = 0; i < horizon; i++) {
            // Get control input for this step
            double[] u;
            if (i < uSequenceReshaped.length) {
                u = uSequenceReshaped[i];
            } else {
                u = uSequenceReshaped[uSequenceReshaped.length - 1];
            }

            // Predict next state using the model's nextState method
            state = predictNextState(state, u);
            time += deltaTime;

            // Get reference for this future timestep
            double[] reference = trajectoryGenerator.getReference(time);

            // Position error cost
            double posError = Math.pow(state[0] - reference[0], 2) + Math.pow(state[1] - reference[1], 2);
            cost += qPos * posError;

            // Orientation error cost (handle angle wrapping)
            double thetaError = Math.pow(angleDiff(reference[2], state[2]), 2);
            cost += qTheta * thetaError;

            // Control effort cost
            cost += r * (Math.pow(u[0], 2) + Math.pow(u[1], 2) + Math.pow(u[2], 2));

            // Optional: Add a smoothness cost by penalizing changes in control inputs
            if (i > 0) {
                double[] prevU = uSequenceReshaped[Math.min(i-1, uSequenceReshaped.length-1)];
                double controlRateX = Math.pow(u[0] - prevU[0], 2);
                double controlRateY = Math.pow(u[1] - prevU[1], 2);
                double controlRateTheta = Math.pow(u[2] - prevU[2], 2);
                cost += 0.05 * (controlRateX + controlRateY + controlRateTheta);
            }
        }

        return cost;
    }

    /**
     * Calculate the smallest angle difference, accounting for wrap-around.
     */
    public double angleDiff(double target, double current) {
        double diff = target - current;
        diff = ((diff + Math.PI) % (2 * Math.PI)) - Math.PI;
        return diff;
    }

    /**
     * Find optimal control sequence using Apache Commons Math.
     */
    public double[][] optimize(double[] currentState, double currentTime) {
        final int controlDimension = 3; // fx, fy, torque
        final int totalDimension = horizon * controlDimension;

        // Create bounds for the optimization variables
        double[] lowerBound = new double[totalDimension];
        double[] upperBound = new double[totalDimension];

        // Set bounds for each control variable
        for (int i = 0; i < totalDimension; i++) {
            if (i % 3 == 0) {
                // Fx bounds
                lowerBound[i] = -maxForceX;
                upperBound[i] = maxForceX;
            } else if (i % 3 == 1) {
                // Fy bounds
                lowerBound[i] = -maxForceY;
                upperBound[i] = maxForceY;
            } else {
                // Torque bounds
                lowerBound[i] = -maxTorque;
                upperBound[i] = maxTorque;
            }
        }

        // Initial guess - can be zeros or based on previous solution
        double[] initialGuess = new double[totalDimension];
        // Optionally, set a better initial guess based on previous solution

        // Create objective function
        MultivariateFunction objectiveFunction = new MultivariateFunction() {
            @Override
            public double value(double[] point) {
                return objective(point, currentState, currentTime);
            }
        };

        // Configure optimizer
        int maxEvaluations = 1000;
        BOBYQAOptimizer optimizer = new BOBYQAOptimizer(2 * totalDimension + 1);

        // Setup optimization problem
        PointValuePair optimum = optimizer.optimize(
                new MaxEval(maxEvaluations),
                new ObjectiveFunction(objectiveFunction),
                GoalType.MINIMIZE,
                new InitialGuess(initialGuess),
                new SimpleBounds(lowerBound, upperBound)
        );

        // Extract optimized control sequence
        double[] optimizedControls = optimum.getPoint();

        // Reshape into 2D array for return
        double[][] controlSequence = new double[horizon][controlDimension];
        for (int i = 0; i < horizon; i++) {
            for (int j = 0; j < controlDimension; j++) {
                controlSequence[i][j] = optimizedControls[i * controlDimension + j];
            }
        }

        return controlSequence;
    }

    /**
     * Alternative optimization method using CMA-ES for potentially better results
     * but at a higher computational cost.
     */
    public double[][] optimizeWithCMAES(double[] currentState, double currentTime) {
        final int controlDimension = 3; // fx, fy, torque
        final int totalDimension = horizon * controlDimension;

        // Create bounds for the optimization variables
        double[] lowerBound = new double[totalDimension];
        double[] upperBound = new double[totalDimension];

        // Set bounds for each control variable
        for (int i = 0; i < totalDimension; i++) {
            if (i % 3 == 0) {
                // Fx bounds
                lowerBound[i] = -maxForceX;
                upperBound[i] = maxForceX;
            } else if (i % 3 == 1) {
                // Fy bounds
                lowerBound[i] = -maxForceY;
                upperBound[i] = maxForceY;
            } else {
                // Torque bounds
                lowerBound[i] = -maxTorque;
                upperBound[i] = maxTorque;
            }
        }

        // Initial guess - zeros or based on previous solution
        double[] initialGuess = new double[totalDimension];

        // Create objective function
        MultivariateFunction objectiveFunction = new MultivariateFunction() {
            @Override
            public double value(double[] point) {
                return objective(point, currentState, currentTime);
            }
        };

        // CMA-ES configuration
        int maxIterations = 1000;
        int maxEvaluations = 10000;
        int populationSize = 4 + (int)(3 * Math.log(totalDimension));
        double[] sigma = new double[totalDimension];
        Arrays.fill(sigma, 0.3); // Initial step size

        // Create and configure CMA-ES optimizer
        CMAESOptimizer optimizer = new CMAESOptimizer(
                maxIterations,
                0.0,  // stopFitness
                true, // isActiveCMA
                0,    // diagonalOnly
                0,    // checkFeasableCount
                new JDKRandomGenerator(),    // random seed (0 = time-based seed)
                true, // generateStatistics
                new SimpleValueChecker(1e-6, 1e-6)
        );

        // Setup optimization problem
        PointValuePair optimum = optimizer.optimize(
                new MaxEval(maxEvaluations),
                new ObjectiveFunction(objectiveFunction),
                GoalType.MINIMIZE,
                new SimpleBounds(lowerBound, upperBound),
                new CMAESOptimizer.PopulationSize(populationSize),
                new CMAESOptimizer.Sigma(sigma),
                new InitialGuess(initialGuess)
        );

        // Extract optimized control sequence
        double[] optimizedControls = optimum.getPoint();

        // Reshape into 2D array for return
        double[][] controlSequence = new double[horizon][controlDimension];
        for (int i = 0; i < horizon; i++) {
            for (int j = 0; j < controlDimension; j++) {
                controlSequence[i][j] = optimizedControls[i * controlDimension + j];
            }
        }

        return controlSequence;
    }

    /**
     * Predict future states based on control sequence.
     */
    public List<double[]> predictTrajectory(double[] currentState, double[][] uSequence) {
        List<double[]> trajectory = new ArrayList<>();
        trajectory.add(Arrays.copyOf(currentState, currentState.length));

        double[] state = Arrays.copyOf(currentState, currentState.length);

        for (int i = 0; i < uSequence.length; i++) {
            // Get control input for this step
            double[] u = uSequence[i];

            // Predict next state and add to trajectory
            state = predictNextState(state, u);
            trajectory.add(Arrays.copyOf(state, state.length));
        }

        return trajectory;
    }

    /**
     * Predict next state using the model.
     */
    private double[] predictNextState(double[] state, double[] u) {
        try {
            // Try using the solver method first if available
            return system.nextStateSolver(state, u, deltaTime);
        } catch (Exception e) {
            // Fall back to regular nextState method
            return system.nextState(state, u, deltaTime);
        }
    }

    /**
     * Apply one control input to the system based on the current state and time
     */
    public double[] applyControl(double[] currentState, double currentTime) {
        // Optimize to get the best control sequence
        double[][] controlSequence = optimize(currentState, currentTime);

        // Apply only the first control input
        double[] optimalControl = controlSequence[0];

        // Apply the control to the model
        return system.stepSolver(optimalControl, deltaTime);
    }
}