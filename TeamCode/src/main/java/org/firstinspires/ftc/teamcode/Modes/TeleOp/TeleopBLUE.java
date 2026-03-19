package org.firstinspires.ftc.teamcode.Modes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    /**
     * This abstract method must be implemented by subclasses to provide the
     * specific AprilTag ID for their alliance.
     *
     * @return The AprilTag ID to target.
     */
    protected abstract int getAprilTagId();
    protected abstract Pose getGoalPose();
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
        shooter = new Shooter(this, getAprilTagId());
        turret = new Turret(this);
        telemetry.addLine("Alliance: " + getAllianceName());
        telemetry.addLine("Targeting AprilTag ID: " + getAprilTagId());
        telemetry.addLine("Ready to start!");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        turret.resetStartPose();
        // --- Main Loop ---
        while (opModeIsActive()) {
            shooter.teleOpController();
            turret.AutoAimingOnError(shooter.getBearing());
            mecanum.teleOp();
            telemetry.update();
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

    @Override
    protected Pose getGoalPose(){return new Pose(17, 132);}
}

// =================================================================================================
// --- 3. The RED Alliance Concrete Class ---
// This class also inherits from TeleOpBase and provides the Red alliance specifics.
// Because it's in the same file as the public TeleopBLUE class, it cannot be public.
// The FTC SDK will still find and list it as an OpMode because of the @TeleOp annotation.
// =================================================================================================

// pos vel dist
// 0.6 1250 1470.6637
// 0 1050 718.0975
// 1 1350 1993.5275
