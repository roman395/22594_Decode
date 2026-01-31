package org.firstinspires.ftc.teamcode.Modes.Autonomos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;

/**
 * Autonomous OpMode that shoots for a duration, then strafes.
 * This has been updated to use the refactored Mecanum, Shooter, and Intake modules.
 */
@Configurable
@Autonomous
public class BlueBigAutoNW extends LinearOpMode {
    Mecanum drive;
    Shooter shooter;
    Intake intake;

    // --- Configuration ---
    public static double drivePower = 1.0;
    public static double driveTimeSec = 1.5; // Time to strafe
    public static double shooterTimeSec = 15.0; // Total time for the shooting phase
    public static double intakePower = 1.0; // Power for the intake feeder
    public static double intakeWaitTimeSec = 2.0; // Wait time between feeding rings
    public static double intakeWorkTimeSec = 0.1; // How long to run the feeder for each ring
    public static int intakeCycles = 6; // How many rings to feed
    public static int APRILTAG_TARGET_ID = 20; // The AprilTag to aim at
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
            // The shooter's internal logic will handle aiming at the AprilTag.
            shooter.autonomousControl(shooterTimeSec * 1000, APRILTAG_TARGET_ID, velocity, pos);

            // This block manages the intake feeder, cycling it on and off.
            if (currentIntakeCycle < intakeCycles) {
                if (isIntakeRunning) {
                    // If the feeder is running, check if it has run long enough.
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
            // Update telemetry
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
        // Strafe until the movement is complete.
        while(opModeIsActive() && !drive.StrafeMove(driveTimeSec * 1000, 1 * drivePower, -1 * drivePower, -1 * drivePower, 1 * drivePower)){
            // Keep the loop running, but do nothing else.
            // The strafeMove method handles the logic.
            telemetry.addData("Strafing Time Left", "%.1f s", driveTimeSec - drive.GetTimer().seconds());
            telemetry.update();
        }

        // --- Cleanup ---
        if (shooter.getVisionPortal() != null) {
            shooter.getVisionPortal().stopStreaming();
            // shooter.getVisionPortal().close(); // Use close() if you are completely done with the camera.
        }

        telemetry.addLine("Autonomous Finished!");
        telemetry.update();
    }
}
