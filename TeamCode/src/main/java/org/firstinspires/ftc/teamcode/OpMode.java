package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import pedro.pathing.follower.Follower;
import pedro.pathing.path.PathChain;
import pedro.pathing.units.AngleUnit;
import pedro.pathing.units.DistanceUnit;
import pedro.system.PedroSystem;
@Autonomous(name = "My Autonomous Path", group = "Competition")
public class MyAutonomousPath extends LinearOpMode {
    private PedroSystem pedroSystem;
    private Follower follower;
    private Paths paths;
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    @Override
    public void runOpMode() {
        initializeHardware();
        initializePedroSystem();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            runAutonomousPaths();
        }
    }
    private void initializeHardware() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializePedroSystem() {
        pedroSystem = new PedroSystem(
                leftFrontMotor,
                leftRearMotor,
                rightFrontMotor,
                rightRearMotor,
                DistanceUnit.INCH,
                AngleUnit.RADIANS
        );
        follower = pedroSystem.getFollower();
        paths = new Paths(follower);
    }

    private void runAutonomousPaths() {
        try {
            telemetry.addData("Status", "Running Path 1");
            telemetry.update();
            follower.followPath(paths.Path1);

            telemetry.addData("Status", "Running Path 2");
            telemetry.update();
            follower.followPath(paths.Path2);

            telemetry.addData("Status", "Running Path 3");
            telemetry.update();
            follower.followPath(paths.Path3);

            telemetry.addData("Status", "Running Path 4");
            telemetry.update();
            follower.followPath(paths.Path4);

            telemetry.addData("Status", "Running Path 5");
            telemetry.update();
            follower.followPath(paths.Path5);

            telemetry.addData("Status", "Running Path 6");
            telemetry.update();
            follower.followPath(paths.Path6);

            telemetry.addData("Status", "All paths completed!");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
        stopMotors();
    }

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
}
