package org.firstinspires.ftc.teamcode.Modes.Autonomos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;

/**
 * Autonomous OpMode that shoots for a duration, then strafes.
 * This has been updated to use the refactored Mecanum, Shooter, and Intake modules.
 */
@Configurable
@Autonomous
public class RedBigAutoWN extends LinearOpMode {
    Mecanum drive;
    Shooter shooter;
    Intake intake;

    // --- Configuration ---
    // Note: drivePower is negative to reverse the strafe direction from the original code.
    public static double drivePower = -1.0;
    public static double driveTimeSec = 1.5;
    public static double shooterTimeSec = 17.0;
    public static double intakePower = 1.0;
    public static double intakeWaitTimeSec = 3.0;
    public static double intakeWorkTimeSec = 0.15;
    public static int intakeCycles = 6;
    public static int APRILTAG_TARGET_ID = 24; // The AprilTag to aim at
    public static double pos = 0.9;
    public static double velocity = 2100;


    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        drive = new Mecanum(this);
        shooter = new Shooter(this, telemetry);
        intake = new Intake(this);

        telemetry.addLine("Robot Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- Autonomous Sequence ---

        // --- 1. Shooting Phase ---
        telemetry.addLine("Starting Shooting Phase...");
        telemetry.update();

        ElapsedTime autonomousTimer = new ElapsedTime();
        ElapsedTime intakeCycleTimer = new ElapsedTime();
        int currentIntakeCycle = 0;
        boolean isIntakeRunning = false;

        // This loop runs for the total duration of the shooting phase.
        while (opModeIsActive() && autonomousTimer.seconds() < shooterTimeSec) {
            // Continuously update the shooter to aim and maintain flywheel speed.
            //shooter.SetCoefficientsAuto(velocity,pos);
            shooter.autonomousControl(shooterTimeSec * 1000, APRILTAG_TARGET_ID, velocity, pos);
            // This block manages the intake feeder, cycling it on and off.
            if (currentIntakeCycle < intakeCycles) {
                if (isIntakeRunning) {
                    // If the feeder i && (!isShootingActive || gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0 || (gamepad.right_trigger - gamepad.left_trigger) != 0)s running, check if it has run long enough.
                    if (intakeCycleTimer.seconds() > intakeWorkTimeSec) {
                        intake.stop(); // Stop the feeder
                        isIntakeRunning = false;
                        intakeCycleTimer.reset(); // Reset for the wait period
                        currentIntakeCycle++;
                    }
                } else {
                    // If the feeder is waiting, check if it has waited long enough.
                    if (intakeCycleTimer.seconds() > intakeWaitTimeSec) {
                        intake.setPower(intakePower); // Start the feeder
                        isIntakeRunning = true;
                        intakeCycleTimer.reset(); // Reset for the work period
                    }
                }
            }
            telemetry.addData("Shooting Time Left", "%.1f s", shooterTimeSec - autonomousTimer.seconds());
            telemetry.addData("Intake Cycles", "%d / %d", currentIntakeCycle, intakeCycles);
            telemetry.update();
        }

        // Ensure all motors are stopped after the shooting phase
        shooter.stopMotors();
        intake.stop();

        // --- 2. Driving Phase ---
        telemetry.addLine("Starting Driving Phase...");
        telemetry.update();

        drive.ResetTimer();
        // Original code used a negative drivePower to reverse strafe direction.
        // Powers for a right strafe: FL:+, FR:-, RL:-, RR:+
        // With drivePower = -1, this becomes: FL:-, FR:+, RL:+, RR:-
        double flPower = 1 * drivePower;
        double frPower = -1 * drivePower;
        double rlPower = -1 * drivePower;
        double rrPower = 1 * drivePower;

        // Strafe until the movement is complete.
        while(opModeIsActive() && !drive.StrafeMove(driveTimeSec * 1000, flPower, frPower, rlPower, rrPower)){
            telemetry.addData("Strafing Time Left", "%.1f s", driveTimeSec - drive.GetTimer().seconds());
            telemetry.update();
        }

        // --- Cleanup ---
        if (shooter.getVisionPortal() != null) {
            shooter.getVisionPortal().stopStreaming();
        }

        telemetry.addLine("Autonomous Finished!");
        telemetry.update();
    }
}
