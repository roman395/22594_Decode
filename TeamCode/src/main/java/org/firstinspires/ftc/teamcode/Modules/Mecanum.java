package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Manages the robot's Mecanum drive chassis.
 * This class handles TeleOp control logic, including a heading stabilization feature (heading lock)
 * using a PID controller. It also provides simple methods for autonomous movements.
 */
@Configurable
public class Mecanum {
    private final DcMotor FL, FR, RL, RR;
    private final Gamepad gamepad;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isAim; // State for toggling heading lock
    public static double targetAngle = 0;

    /**
     * PIDF Coefficients for the heading stabilization PID controller.
     * p (proportional): corrects the current error.
     * i (integral): corrects for accumulated steady-state error.
     * d (derivative): predicts and dampens future error.
     * f (feedforward): provides a constant power to overcome static friction.
     */
    public static PIDFCoefficients pidf = new PIDFCoefficients(0.015, 0.1, 0.002, 0);
    
    // The rotational power component, calculated either by manual input or the PID controller.
    private double rx = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // --- PID state variables ---
    private double integralSum = 0;
    private double previous_error = 0;
    private double previous_time = 0;
    private boolean wasAiming = false; // Tracks if PID was active in the previous loop cycle.
    private static final double INTEGRAL_SUM_LIMIT = 0.25; // Anti-windup limit for the integral term

    /**
     * Initializes the Mecanum drive module.
     * @param lom The LinearOpMode instance to access hardwareMap and gamepad.
     */
    public Mecanum(LinearOpMode lom) {
        // Hardware Mapping
        FL = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumFL);
        FR = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumFR);
        RL = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumRL);
        RR = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumRR);

        // Set Zero Power Behavior to BRAKE
        // This makes the robot stop more abruptly when power is set to zero.
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Motor Directions as per user's configuration
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize dependencies
        gamepad = lom.gamepad1;
        pidTimer.reset();
    }

    /**
     * Main control loop for TeleOp. Call this repeatedly in your opMode's loop().
     * @param angle The current robot heading from an IMU, in degrees.
     * @param t The Telemetry object for displaying debug info.
     */
    public void TeleOp(double angle, Telemetry t) {
        // Toggle heading lock mode on/off when the 'cross' button is pressed.
        if (gamepad.crossWasPressed()) {
            isAim = !isAim;
        }

        // --- Get Driver Inputs ---
        // Y-stick is inverted for standard "forward" control.
        double y = -gamepad.left_stick_y;
        // Apply a correction factor for strafing if needed.
        double x = gamepad.left_stick_x * 1.1; 

        // --- Determine Rotational Power (Manual vs. PID) ---
        // Use PID if aiming is enabled, a valid angle is provided, and the driver is not manually turning.
        boolean usePid = isAim && angle != -999 && Math.abs(gamepad.right_trigger - gamepad.left_trigger) <= 0.1;

        if (usePid) {
            // If we are just starting to aim, reset the PID state to avoid using stale values.
            if (!wasAiming) {
                integralSum = 0;
                previous_error = 0;
                pidTimer.reset();
                previous_time = 0;
                wasAiming = true;
            }
            // Update the rotational power (rx) using the PID controller.
            PID(angle);
            t.addData("Current Angle", "%.2f", angle);
        } else {
            // Use manual input from triggers for rotation.
            rx = (gamepad.right_trigger - gamepad.left_trigger) * 0.9;
            wasAiming = false;
        }

        // --- Mecanum Drive Kinematics ---
        // The denominator is the largest motor power (absolute value) or 1.
        // This ensures all powers maintain the same ratio, but scales them down
        // only if the total power for any wheel would exceed 1.0.
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double rlPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double rrPower = (y + x - rx) / denominator;

        // Apply the calculated powers to the motors.
        setMotorPowers(flPower, frPower, rlPower, rrPower);
        t.addData("Heading Lock (Aim)", isAim);
    }

    /** Resets the internal timer used for autonomous movements. */
    public void ResetTimer() {
        timer.reset();
    }

    /** Returns the ElapsedTime object for autonomous movements. */
    public ElapsedTime GetTimer() {
        return timer;
    }

    /**
     * Moves the robot forward or backward with a given power for a specified time.
     * @param time The duration of the movement in milliseconds.
     * @param power The motor power [-1.0, 1.0]. Positive for forward, negative for backward.
     * @return True when the movement is complete, otherwise false.
     */
    public boolean ForwardMove(double time, double power) {
        if (time > timer.milliseconds()) {
            setMotorPowers(power, power, power, power);
        } else {
            setMotorPowers(0, 0, 0, 0);
            return true;
        }
        return false;
    }

    /**
     * Strafes the robot with given motor powers for a specified time.
     * @param time The duration of the movement in milliseconds.
     * @param flPower Power for the Front-Left motor.
     * @param frPower Power for the Front-Right motor.
     * @param rlPower Power for the Rear-Left motor.
     * @param rrPower Power for the Rear-Right motor.
     * @return True when the movement is complete, otherwise false.
     */
    public boolean StrafeMove(double time, double flPower, double frPower, double rlPower, double rrPower) {
        if (time > timer.milliseconds()) {
            setMotorPowers(flPower, frPower, rlPower, rrPower);
        } else {
            setMotorPowers(0, 0, 0, 0);
            return true;
        }
        return false;
    }

    /**
     * A helper method to set the power for all four mecanum wheel motors.
     */
    private void setMotorPowers(double flPower, double frPower, double rlPower, double rrPower) {
        FL.setPower(flPower);
        FR.setPower(frPower);
        RL.setPower(rlPower);
        RR.setPower(rrPower);
    }

    /**
     * Calculates the required rotational power using a PID controller to hold a target heading.
     * The output is stored in the `rx` variable.
     * @param angle The current robot heading.
     */
    private void PID(double angle) {
        double current_time = pidTimer.milliseconds();
        double dt = current_time - previous_time;
        
        // Ensure time has passed to prevent division by zero.
        if (dt <= 0) { 
            previous_time = current_time;
            return;
        }
        
        // Calculate error (difference between target and current heading).
        double current_error = targetAngle - angle;

        // P (Proportional) - Responds to the current error.
        double p = pidf.p * current_error;

        // I (Integral) - Accumulates error over time to correct for steady-state drift.
        integralSum += current_error * dt;
        // Anti-windup: Clamp the integral sum to prevent it from growing too large.
        if (integralSum > INTEGRAL_SUM_LIMIT) integralSum = INTEGRAL_SUM_LIMIT;
        if (integralSum < -INTEGRAL_SUM_LIMIT) integralSum = -INTEGRAL_SUM_LIMIT;
        double i = pidf.i * integralSum;

        // D (Derivative) - Dampens oscillation by considering the rate of change of the error.
        double derivative = (current_error - previous_error) / dt;
        double d = pidf.d * derivative;

        // F (Feedforward) - Applies a constant force to overcome static friction, proportional to the sign of the error.
        double f = pidf.f * Math.signum(current_error);

        // Sum all components to get the final rotational power.
        rx = p + i + d + f;

        // Save the current error and time for the next loop's derivative calculation.
        previous_error = current_error;
        previous_time = current_time;
    }
}
