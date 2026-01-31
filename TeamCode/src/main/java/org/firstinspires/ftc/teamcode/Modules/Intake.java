package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;

/**
 * Manages the robot's intake and feeder mechanism.
 * This class provides simple, direct control over the intake motor for both autonomous and tele-op.
 */
public class Intake {

    // --- Configuration --- //
    /**
     * Static inner class for tunable power parameters.
     */
    public static class Configuration {
        /** The default power for running the intake forward (e.g., collecting pixels). */
        public static final double FORWARD_POWER = 1.0;
        /** The default power for running the intake in reverse. */
        public static final double REVERSE_POWER = -1.0;
    }

    // --- Hardware Components --- //
    private final DcMotor intakeMotor;
    private final Gamepad gamepad;
    /**
     * Initializes the Intake module.
     * @param lom The LinearOpMode instance to access hardwareMap.
     */
    public Intake(LinearOpMode lom) {
        // Hardware Mapping
        intakeMotor = lom.hardwareMap.get(DcMotor.class, RobotConstants.IntakeMotor);
        gamepad = lom.gamepad1;
        // Set Motor Direction and Behavior
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the power of the intake motor directly.
     * This is the primary method for controlling the intake from other modules or OpModes.
     *
     * @param power The power to set the motor to, in the range [-1.0, 1.0].
     *              Positive values should run the intake forward (collecting).
     *              Negative values should run it in reverse.
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * A convenience method to run the intake forward at the default power.
     */
    public void run() {
        setPower(Configuration.FORWARD_POWER);
    }

    /**
     * A convenience method to run the intake in reverse at the default power.
     */
    public void reverse() {
        setPower(Configuration.REVERSE_POWER);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        setPower(0);
    }
    
    /**
     * Alias for setPower(), kept for compatibility with the Shooter module's previous implementation.
     * @param power The power to set the motor to.
     */
    public void ShooterEnable(double power) {
        setPower(power);
    }
    public double getPower(){return intakeMotor.getPower();}
    public void shooterControl(double power){
        if(gamepad.right_bumper)
            setPower(power);
        else if(gamepad.left_bumper)
            setPower(-power);
        else
            setPower(0);
    }
}
