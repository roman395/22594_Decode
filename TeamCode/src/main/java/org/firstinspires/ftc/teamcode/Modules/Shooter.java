package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Camera.AprilTagsDetection;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

/**
 * Manages the robot's shooter mechanism, including flywheel velocity control, aiming, and ring feeding.
 * It uses a custom PIDF controller for precise velocity management and can leverage AprilTags for automated aiming.
 */
@Configurable
public class Shooter {

    // --- Configuration --- //

    /**
     * Static inner class for tunable parameters. Grouping them here makes tuning easier.
     */
    @Configurable
    public static class Configuration {
        // PID and Feed-Forward coefficients for the flywheel velocity controller.
        public static PIDFCoefficients PID_COEFFICIENTS = new PIDFCoefficients(0.1, 0, 0.005, 0);
        // Base power to counteract friction and inertia. This helps the flywheel reach the target speed faster.
        public static double FEED_FORWARD = 0;

        // Manual control parameters for when auto-aim is disabled.
        public static double MANUAL_VELOCITY = 1800; // Default velocity in ticks/sec.
        public static double MANUAL_SERVO_POSITION = 0.5; // Default servo angle.

        // Behavior flags.
        public static boolean USE_CAMERA_AIM = true; // Master switch for enabling/disabling auto-aim.
        public static boolean USE_PHYSICS_MODEL = false; // Switch for an alternative (currently unused) physics-based model.

        // PID Controller limits.
        private static final double MAX_POWER = 0.99; // Prevents motor damage by capping power.
        private static final double INTEGRAL_SUM_LIMIT = 0.5; // Prevents integral wind-up for stability.
    }

    // --- Hardware Components --- //
    private final DcMotorEx leftMotor, rightMotor;
    private final Servo servo;
    private final ExposureControl cameraExposureControl;

    // --- Dependencies & State --- //
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    public final AprilTagsDetection aprilTagDetector;
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime autonomousActionTimer = new ElapsedTime(); // New timer
    public static double criticalErrorFar = 200, criticalErrorClose = 20, farDistance = 2460;
    private double servoPos = 0;

    private boolean isShootingActive = false;
    private boolean hasIntakeRun = false;
    private boolean oldApprox = false;
    private double currentTargetVelocity = Configuration.MANUAL_VELOCITY;
    private double currentTargetServoPosition = Configuration.MANUAL_SERVO_POSITION;

    // New state for autonomous shooting sequence
    private enum AutonomousShootingState {IDLE, AIMING_AND_SPOOLING, FEEDING_RING, WAITING_FOR_RECOVERY, DONE}

    private AutonomousShootingState autonomousState = AutonomousShootingState.IDLE;
    private int autonomousShotsFired = 0;


    // --- PID Control State --- //
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double currentError = 0.0;
    private double pidOutput = 0.0;
    private double distance = 0;
    private double intakePower = 0;

    /**
     * Initializes the Shooter module, its hardware components, and dependencies.
     *
     * @param lom The LinearOpMode instance, used to access hardwareMap and gamepad.
     * @param tel The Telemetry instance for displaying debug information on the Driver Hub.
     */
    public Shooter(LinearOpMode lom, Telemetry tel) {
        this.gamepad = lom.gamepad1;
        this.telemetry = tel;

        // Initialize motors
        leftMotor = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootLeft);
        rightMotor = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootRight);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // One motor must spin opposite to the other.

        // Set motor modes
        // FLOAT allows flywheels to spin down freely, which is often desired for shooters.
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // We use a custom PID, so we run without the built-in motor PID.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos for aiming
        servo = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServo);
        servo.setDirection(Servo.Direction.REVERSE); // Reverse one servo to make them mirror each other.

        // Initialize camera for AprilTag detection
        aprilTagDetector = new AprilTagsDetection(lom, tel);
        getVisionPortal().resumeStreaming();
        PanelsCameraStream.INSTANCE.startStream(getVisionPortal(), 30);
        // Configure camera exposure for better AprilTag detection, especially in bright environments.
        cameraExposureControl = getVisionPortal().getCameraControl(ExposureControl.class);
        cameraExposureControl.setMode(ExposureControl.Mode.Manual);
        cameraExposureControl.setExposure(3, TimeUnit.MILLISECONDS);
    }

    /**
     * Main control loop for TeleOp. Manages aiming, shooting, and feeding based on driver input.
     *
     * @param intake   The Intake module, used for feeding rings into the shooter.
     * @param targetId The AprilTag ID to aim at.
     * @return The bearing (horizontal angle) to the target AprilTag, for use in chassis alignment.
     */
    public double teleOpControl(Intake intake,Turret turret, int targetId) {
        aprilTagDetector.Update(); // Refresh camera data every loop
        intakePower = intake.getPower();
        turret.AutoAiming(aprilTagDetector.GetBearing(targetId));
        updateTargets(targetId);
        handleShootingAndFeeding(intake);
        handleMotorPower();
        handleManualAdjustments();
        handleServo();
        displayTelemetry(targetId);
        if(gamepad.startWasPressed())
            Configuration.USE_CAMERA_AIM = !Configuration.USE_CAMERA_AIM;
        if(gamepad.optionsWasPressed())
            oldApprox = !oldApprox;
        return aprilTagDetector.GetBearing(targetId);
    }

    /**
     * Main control loop for Autonomous. Aims and runs the shooter for a specified duration.
     *
     * @param time     The time in milliseconds to run the shooter.
     * @param targetId The AprilTag ID to aim at.
     * @return True if the autonomous action is complete, otherwise false.
     */
    public void autonomousControl(double time, int targetId, double velocity, double position) {
        //aprilTagDetector.Update();

        servo.setPosition(position);
        if (time > shooterTimer.milliseconds()) {
            updatePID(velocity);
        } else {
            stopMotors();
        }
    }

    /**
     * Manages an autonomous shooting sequence for use with libraries like Pedro Pathing.
     * This method should be called repeatedly in a loop until it returns true.
     * It handles aiming, spooling up the flywheel, feeding a ring, and stopping.
     *
     * @param intake   The intake module, used to feed the ring.
     * @param targetId The AprilTag ID to aim and shoot at.
     * @return {@code true} when the entire shooting sequence is complete.
     */
    public boolean runAutonomousShootingSequence(Intake intake, int targetId) {
        aprilTagDetector.Update();
        if(Configuration.USE_CAMERA_AIM)
            updateTargets(targetId);
        displayTelemetry(targetId);
        switch (autonomousState) {
            case IDLE:
                // When the sequence starts, reset timers and shot counter, then move to the next state.
                resetPID();
                autonomousShotsFired = 0;
                autonomousActionTimer.reset();
                autonomousState = AutonomousShootingState.AIMING_AND_SPOOLING;
                return false;

            case AIMING_AND_SPOOLING:
                // Step 1: Aim servos and spool up the flywheel.
                updatePID(currentTargetVelocity);
                // Check if the flywheel is at the correct speed before the first shot.
                if (Math.abs(lastError) < criticalErrorClose ) {
                    autonomousActionTimer.reset();
                    autonomousState = AutonomousShootingState.FEEDING_RING;
                }
                return false;

            case FEEDING_RING:
                // Step 2: Continue running the flywheel and feed one ring.
                updatePID(currentTargetVelocity);
                intake.ShooterEnable(1);

                // Wait until the error increases (indicating a shot) or a timeout is reached.
                if (Math.abs(lastError) > criticalErrorClose || autonomousActionTimer.seconds() > 1.5) {
                    intake.ShooterEnable(0);
                    autonomousShotsFired++;

                    if (autonomousShotsFired >= 3) {
                        autonomousState = AutonomousShootingState.DONE;
                    } else {
                        autonomousState = AutonomousShootingState.WAITING_FOR_RECOVERY;
                    }
                }
                return false;

            case WAITING_FOR_RECOVERY:
                // Step 3: Keep the flywheel spooled and wait for the velocity to stabilize.
                updatePID(currentTargetVelocity);
                intake.ShooterEnable(0); // Ensure feeder is off.

                // Once the speed is back within the threshold, reset the timer and feed the next ring.
                if (Math.abs(lastError) < criticalErrorClose) {
                    autonomousActionTimer.reset();
                    autonomousState = AutonomousShootingState.FEEDING_RING;
                }
                return false;

            case DONE:
                // The sequence is complete. Stop all motors.
                stopMotors();
                intake.ShooterEnable(0);
                return true;
        }
        return true; // Should be unreachable, but indicates completion.
    }

    /**
     * Resets the autonomous shooting sequence state machine, allowing it to be run again.
     */
    public void resetAutonomousShootingSequence() {
        autonomousState = AutonomousShootingState.IDLE;
        autonomousShotsFired = 0;
        stopMotors();
    }

    /**
     * Reverses the shooter motors to prevent pixels from falling out.
     *
     * @param power The power to apply to the motors (should be negative).
     */
    public void reverse(double power) {
        // Ensure power is negative
        power = -Math.abs(power);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        resetPID();
    }

    /**
     * A test method for tuning PID and regression functions manually.
     *
     * @return The current PID error.
     */
    public double velocityTests(double velocity, double pos, int id, boolean isCameraUse) {
        aprilTagDetector.Update();
        telemetry.addData("distance", aprilTagDetector.GetDistance(id));
        telemetry.addData("Shooter target velocity", velocity);

        double distance = aprilTagDetector.GetDistance(id);

        updatePID(velocity);
        servo.setPosition(pos);
        displayTelemetry(id);
        return distance;
    }

    // --- Helper Methods for Control Logic --- //

    private void updateTargets(int targetId) {
        if (Configuration.USE_CAMERA_AIM) {
            distance = aprilTagDetector.GetDistance(targetId);
            // Only update targets if we have a valid detection.
            // The extra check prevents the aim from changing while the driver is stationary.
            if (distance != -1) {
                currentTargetVelocity = distToVelocity(distance);
                currentTargetServoPosition = distToWallPose(distance);
            }
        } else {
            // In manual mode, velocity is set by D-pad.
            currentTargetVelocity = Configuration.MANUAL_VELOCITY;
        }
        servoPos = currentTargetServoPosition;
        // Apply servo position
    }

    private void handleShootingAndFeeding(Intake intake) {
        // Start/Stop shooting
        if (gamepad.circle) {
            isShootingActive = true;
        } else if (gamepad.square) {
            isShootingActive = false;
        }

        // Reset timer when bumper is released to control the post-shot reversal duration.
        if (gamepad.rightBumperWasReleased()) shooterTimer.reset();
        if (gamepad.rightBumperWasPressed() && !hasIntakeRun) hasIntakeRun = true;

        // Determine if we should feed a ring
        boolean shouldFeed = false;
        // Allow pre-loading rings before spinning up the shooter.
        if (gamepad.right_bumper && !isShootingActive) {
            shouldFeed = true;
        }
        // Only feed when shooting if the flywheel speed has stabilized (low PID error).
        // This is critical for consistent shot velocity.
        if (distance < farDistance && isShootingActive && gamepad.right_bumper && Math.abs(currentError) < criticalErrorFar) {
            shouldFeed = true;
        } else if (isShootingActive && gamepad.right_bumper && Math.abs(currentError) < criticalErrorClose) {
            shouldFeed = true;
        }

        // Control the intake based on the feeding logic
        if (shouldFeed) {
            intake.ShooterEnable(1); // Run feeder forward
        } else if (gamepad.left_bumper) {
            intake.ShooterEnable(-0.6); // Run feeder in reverse
        } else {
            intake.ShooterEnable(0); // Stop feeder
        }
    }

    private void handleMotorPower() {
        if (isShootingActive) {
            // Maintain target velocity using PID.
            updatePID(currentTargetVelocity);
        }
        // After a shot, briefly reverse motors to prevent rings from getting stuck.
        else if ((gamepad.right_bumper || shooterTimer.milliseconds() < 1500) && hasIntakeRun) {
            reverse(0.6);
            resetPID(); // Reset PID so integral doesn't accumulate during reversal.
        }
        // Manual reverse for un-jamming.
        else if (gamepad.triangle) {
            reverse(0.5);
            resetPID();
        }
        // If no other action is active, turn off the motors.
        else {
            stopMotors();
        }
    }

    private void handleServo() {
        servo.setPosition(servoPos);
    }

    private void handleManualAdjustments() {
        if (!Configuration.USE_CAMERA_AIM) {
            gamepad.setLedColor(255, 0, 0, 1000); // Red LED for manual mode
            if (gamepad.dpadLeftWasPressed()) Configuration.MANUAL_VELOCITY -= 50;
            if (gamepad.dpadRightWasPressed()) Configuration.MANUAL_VELOCITY += 50;
            if (gamepad.dpadUpWasPressed()) currentTargetServoPosition += 0.05;
            if (gamepad.dpadDownWasPressed()) currentTargetServoPosition -= 0.05;
        } else {
            gamepad.setLedColor(0, 255, 0, 1000); // Green LED for auto-aim mode
        }
    }

    // --- PID Controller --- //

    /**
     * Updates the flywheel motor powers using a PIDF controller to maintain a target velocity.
     *
     * @param targetVelocity The desired velocity in ticks per second.
     */
    private void updatePID(double targetVelocity) {
        if (targetVelocity <= 0) {
            stopMotors();
            return;
        }

        // 1. Get current data
        double current_time = pidTimer.milliseconds();
        double currentVelocity = (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2.0;
        currentError = targetVelocity - currentVelocity;

        // 2. Calculate time change; skip loop if delta is too small to be reliable.
        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) {
            return; // Avoids division by zero and unstable derivative calculations.
        }

        // 3. Calculate P, I, D components
        double pComponent = Configuration.PID_COEFFICIENTS.p * currentError;

        integralSum += (currentError * deltaTime);
        integralSum = Math.max(-Configuration.INTEGRAL_SUM_LIMIT, Math.min(integralSum, Configuration.INTEGRAL_SUM_LIMIT)); // Anti-windup
        double iComponent = Configuration.PID_COEFFICIENTS.i * integralSum;

        double derivative = (currentError - lastError) / deltaTime;
        double dComponent = Configuration.PID_COEFFICIENTS.d * derivative;

        // 4. Add Feed-Forward (F) component
        double feedForwardPower = Configuration.FEED_FORWARD;

        // 5. Sum all components to get the final output power.
        pidOutput = feedForwardPower + pComponent + iComponent + dComponent;

        // 6. Clamp output power to a safe range.
        pidOutput = Math.max(0, Math.min(pidOutput, Configuration.MAX_POWER));

        // 7. Apply power to motors.
        rightMotor.setPower(pidOutput);
        leftMotor.setPower(pidOutput);

        servo.setPosition(servoPos);
        // 8. Save state for the next loop iteration.
        lastError = currentError;
        lastTime = current_time;
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = 0;
        pidOutput = 0;
    }

    // --- Utility Methods --- //

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetPID();
    }

    public void resetShooterTimer() {
        shooterTimer.reset();
    }

    private void displayTelemetry(int tagId) {
        if (Configuration.USE_CAMERA_AIM) {
            telemetry.addData("Distance to Target", "%.2f in", aprilTagDetector.GetDistance(24));
        }
        telemetry.addData("Shooter velocity", leftMotor.getVelocity());
        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Wall pos", servoPos);
        telemetry.addData("April tag detection", aprilTagDetector.GetDistance(tagId) != -1);
        telemetry.addData("Error", currentError);
    }

    /**
     * Provides access to the VisionPortal instance for external use (e.g., streaming).
     */
    public VisionPortal getVisionPortal() {
        return aprilTagDetector.getPortal();
    }

    // --- Regression & Physics Functions --- //

    /**
     * Calculates flywheel velocity from distance using a regression function derived from testing.
     */
    private double distToVelocityNew(double dist) {
        return -0.00000832850904453064 * dist * dist * 0 +0.1595  * dist + 1610.6;
        //0.16237677658523352875 1729.16030889091780409217
    }
    private double distToVelocityOld(double dist) {
        return -0.00000832850904453064 * dist * dist + 0.16237677658523352875  * dist + 1729.16;
        //0.16237677658523352875 1729.16030889091780409217
    }

    /**
     * Calculates the wall/deflector servo position from distance using a regression function.
     */
    private double distToWallPoseNew(double dist) {
        double position = 0.00035929030903905293 * dist - 0.24707690788092798173;
    //return Math.min(position, 0.95); // Cap servo position to a safe maximum.
        return 0.9;
    }
    private double distToWallPoseOld(double dist) {
        double position = 0.00035929030903905293 * dist - 0.24707690788092798173;
        return Math.min(position, 0.95); // Cap servo position to a safe maximum.
    }
    private double distToWallPose(double dist){
        if(oldApprox)
            return distToWallPoseOld(dist);
        else
            return distToWallPoseNew(dist);
    }
    private double distToVelocity(double dist){
        if(oldApprox)
            return distToVelocityOld(dist);
        else
            return distToVelocityNew(dist);
    }
    /*
     * The following methods appear to be for a physics-based model that is not currently used.
     * They are preserved for future reference.
     */
    private double distToLaunchAngle(double elevation) {
        return Math.atan(Math.tan(Math.toRadians(elevation)));
    }

    private double launchAngleToServo(double angle) {
        return Math.toDegrees(angle) / 180 * 0.95;
    }

    private double distToVelocityPhysic(double dist, double launchAngle) {
        return Math.sqrt(9.81 * dist / Math.sin(2 * launchAngle));
    }

    private double velocityToTPS(double velocity) {
        return (28 * 28 / 22.0 * velocity) / (Math.PI * 0.072);
    }
    public void useOldApprox(boolean toggle){oldApprox = toggle;}
    public void SetCoefficientsAuto(double velocity, double pos){
        servoPos = pos;
        currentTargetVelocity = velocity;
        Configuration.USE_CAMERA_AIM = false;
    }
}
