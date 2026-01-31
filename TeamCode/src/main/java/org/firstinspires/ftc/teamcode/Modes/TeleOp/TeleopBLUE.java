package org.firstinspires.ftc.teamcode.Modes.TeleOp;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;

// =================================================================================================
// --- 1. The Abstract Base Class ---
// This class contains all the common logic for both Red and Blue TeleOp modes.
// It is marked as @Disabled so it doesn't appear in the OpMode list on the Driver Station.
// =================================================================================================
@Disabled
abstract class TeleOpBase extends LinearOpMode {
    // --- Modules ---
    Mecanum mecanum;
    Intake intake;
    Shooter shooter;
    Turret turret;
    Telemetry tel = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    /**
     * This abstract method must be implemented by subclasses to provide the
     * specific AprilTag ID for their alliance.
     *
     * @return The AprilTag ID to target.
     */
    protected abstract int getAprilTagId();

    /**
     * This abstract method provides the name of the alliance for telemetry.
     *
     * @return The alliance name as a String.
     */
    protected abstract String getAllianceName();

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        mecanum = new Mecanum(this);
        intake = new Intake(this);
        shooter = new Shooter(this, tel);
        turret = new Turret(this);
        tel.addLine("Alliance: " + getAllianceName());
        tel.addLine("Targeting AprilTag ID: " + getAprilTagId());
        tel.addLine("Ready to start!");
        tel.update();
        shooter.useOldApprox(false);
        waitForStart();

        if (isStopRequested()) return;

        // --- Main Loop ---
        while (opModeIsActive()) {
            // The shooter's control method returns the bearing to the AprilTag target.
            double bearingToTarget = shooter.teleOpControl(intake, turret, getAprilTagId());

            // The mecanum's control method uses this bearing as the input for its heading PID controller,
            // which will try to turn the robot to make the bearing zero.
            mecanum.TeleOp(bearingToTarget, tel);

            // Telemetry is updated within the modules, but an extra update here is fine.
            tel.update();
        }

        // --- Cleanup ---
        if (shooter.getVisionPortal() != null) {
            shooter.getVisionPortal().stopStreaming();
        }
    }
}

// =================================================================================================
// --- 2. The BLUE Alliance Concrete Class ---
// This class inherits all the logic from TeleOpBase and just provides the Blue alliance specifics.
// =================================================================================================
@TeleOp(name = "TeleOp BLUE", group = "Main")
public class TeleopBLUE extends TeleOpBase {
    @Override
    protected int getAprilTagId() {
        return 20;
    }

    @Override
    protected String getAllianceName() {
        return "BLUE";
    }
}

// =================================================================================================
// --- 3. The RED Alliance Concrete Class ---
// This class also inherits from TeleOpBase and provides the Red alliance specifics.
// Because it's in the same file as the public TeleopBLUE class, it cannot be public.
// The FTC SDK will still find and list it as an OpMode because of the @TeleOp annotation.
// =================================================================================================

