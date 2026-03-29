package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Camera.AprilTagsDetection;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public class Shooter {
    public static double gangBangError = 100;
    boolean notUseFish = false;
    AprilTagsDetection camera;
    Telemetry telemetry;
    Telemetry tel = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    private boolean cameraActive = true;
    private boolean handleControl = false;
    private final DcMotorEx slaveMotor, masterMotor;
    private final Servo wallServo;
    private int aprilTagId = 0;
    private final Gamepad gamepad;
    private final ElapsedTime LEDTimer = new ElapsedTime();
    private final ElapsedTime autonomTimer = new ElapsedTime();
    public static int LEDDuration = 1000;
    private boolean LEDState = false;
    private double automatedVelocity = 0;
    private double automatedServoPose = 0.1;
    private double bearing = 0;
    private double distance = 0;

    public enum STATE {OFF, SPOILING, SHOOTING}

    STATE state = STATE.OFF;
    public final Intake intakeModule;
    private final Turret turret;
    private Limelight3A limelight3A;
    private LLResult result;
    public static boolean useLogitech = false;

    public void controllerLED() {
        if (!cameraActive) {
            if (LEDState && LEDTimer.milliseconds() > LEDDuration) {
                LEDState = false;
                gamepad.setLedColor(0, 0, 0, LEDDuration);
                LEDTimer.reset();
            } else if (!LEDState && LEDTimer.milliseconds() > LEDDuration) {
                LEDState = true;
                gamepad.setLedColor(255, 0, 0, LEDDuration);
                LEDTimer.reset();
            }
        } else if (handleControl) {
            if (LEDState && LEDTimer.milliseconds() > LEDDuration) {
                LEDState = false;
                gamepad.setLedColor(0, 0, 0, LEDDuration);
                LEDTimer.reset();
            } else if (!LEDState && LEDTimer.milliseconds() > LEDDuration) {
                LEDState = true;
                gamepad.setLedColor(255, 255, 0, LEDDuration);
                LEDTimer.reset();
            }
        } else {
            if (LEDState && LEDTimer.milliseconds() > LEDDuration) {
                LEDState = false;
                gamepad.setLedColor(0, 0, 0, LEDDuration);
                LEDTimer.reset();
            } else if (!LEDState && LEDTimer.milliseconds() > LEDDuration) {
                LEDState = true;
                gamepad.setLedColor(0, 255, 0, LEDDuration);
                LEDTimer.reset();
            }
        }
    }

    public void CameraInitialization(LinearOpMode linearOpMode) {
        if (useLogitech) {
            camera = new AprilTagsDetection(linearOpMode, telemetry);
            if (camera.getPortal().getCameraState() == VisionPortal.CameraState.ERROR) {
                cameraActive = false;
                handleControl = true;
                return;
            }
            camera.getPortal().resumeStreaming();
            PanelsCameraStream.INSTANCE.startStream(camera.getPortal(), 60);

            ExposureControl exposureControl = camera.getPortal().getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(RobotConstants.CameraExposure, TimeUnit.MILLISECONDS);
        } else {
            limelight3A = linearOpMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight3A.start();
            if (aprilTagId == 20) {
                limelight3A.pipelineSwitch(0);
                tel.addLine("LimeLight ready");
            } else if (aprilTagId == 24) {
                limelight3A.pipelineSwitch(1);
                tel.addLine("LimeLight ready");
            } else {
                limelight3A.pipelineSwitch(2);
                tel.addLine("LimeLight ready");
            }
            limelight3A.reloadPipeline();
        }
    }

    public Shooter(LinearOpMode linearOpMode, int aprilTagId, Pose goal) {
        masterMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootRight);
        slaveMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootLeft);

        masterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slaveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        masterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slaveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        masterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slaveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wallServo = linearOpMode.hardwareMap.get(Servo.class, RobotConstants.ShootServo);
        wallServo.setDirection(Servo.Direction.REVERSE);
        this.aprilTagId = aprilTagId;
        CameraInitialization(linearOpMode);
        gamepad = linearOpMode.gamepad1;
        intakeModule = new Intake(linearOpMode);
        turret = new Turret(linearOpMode, goal);
        telemetry = linearOpMode.telemetry;
    }

    public void resetAll() {
        resetPID();
        LEDTimer.reset();
    }

    public void teleOpController() {
        controllerLED();
        if (gamepad.startWasPressed())
            handleControl = !handleControl;
        if (gamepad.optionsWasPressed())
            notUseFish = !notUseFish;
        if (useLogitech)
            camera.update();
        updateTarget();
        updateWall(automatedServoPose);
        if (gamepad.squareWasPressed())
            state = STATE.OFF;
        switch (state) {
            case OFF:
                intakeModule.controlUntilShooting(gamepad, notUseFish);
                if (gamepad.circleWasPressed())
                    state = STATE.SPOILING;
                updatePID(0, masterMotor, slaveMotor);
                break;
            case SPOILING:
                intakeModule.controlUntilShooting(gamepad, notUseFish);
                updatePID(automatedVelocity, masterMotor, slaveMotor);
                if (isCanShooting(distance))
                    state = STATE.SHOOTING;
                break;
            case SHOOTING:
                intakeModule.intakeControl(gamepad);
                intakeModule.feedingControl(gamepad);
                updatePID(automatedVelocity, masterMotor, slaveMotor);
                if (!isCanShooting(distance))
                    state = STATE.SPOILING;
                break;
        }
    }

    public void testing(double shooterVelocity, double wallPose) {
        updatePID(shooterVelocity, masterMotor, slaveMotor);
        updateWall(Math.clamp(wallPose, 0.1, 1));
        updateTarget();
        intakeModule.feedingControl(gamepad);
        intakeModule.intakeControl(gamepad);
    }

    int cntShoot = 0;
    int targetCntShoot = 3;

    public boolean runAutonomousShootingSequence() {
        updateTarget();

        switch (state) {
            case OFF:
                state = STATE.SPOILING;
                break;
            case SPOILING:
                intakeModule.stopAll();
                updatePID(automatedVelocity, masterMotor, slaveMotor);
                updateWall(automatedServoPose);

                if (cntShoot < targetCntShoot && isCanShooting(distance)) {
                    state = STATE.SHOOTING;
                    autonomTimer.reset();
                }

                if (cntShoot > targetCntShoot - 1 || autonomTimer.milliseconds() > 2000) {
                    shooterOff();
                    intakeModule.stopAll();
                    return true;
                }
                break;
            case SHOOTING:
                updatePID(automatedVelocity, masterMotor, slaveMotor);
                intakeModule.enableAll();
                if (currentError > 200 || autonomTimer.milliseconds() > 700) {
                    cntShoot++;
                    intakeModule.stopAll();
                    autonomTimer.reset();
                    state = STATE.SPOILING;
                }
                break;
        }

        return false;
    }

    public STATE getState() {
        return state;
    }
    private double spoilingVelocity;
    public void startSpooling(double velocity) {
        spoilingVelocity = velocity;
    }
    public void updateSpooling(){
        updatePID(spoilingVelocity, masterMotor, slaveMotor);
    }

    private void shooterOff() {
        masterMotor.setPower(0);
        slaveMotor.setPower(0);
    }

    public void resetAutonomousShootingSequence() {
        state = STATE.OFF;
        automatedVelocity = 1600;
        automatedServoPose = 0.5;
        cntShoot = 0;
        resetPID();
        autonomTimer.reset();
    }

    public void advancedTelemetry(Telemetry telemetry) {
        boolean turretAligned = Math.abs(bearing) < 2.0 && bearing != -999999;
        boolean velocityReady = Math.abs(currentError) < 20;

        telemetry.addLine("--- SHOOTER DIAGNOSTICS ---");
        telemetry.addData("State", state);
        telemetry.addData("Shots Done", cntShoot + "/" + targetCntShoot);
        telemetry.addData("Distance", distance);
        telemetry.addData("Bearing", bearing);
        telemetry.addData("1. Turret Aligned", turretAligned);
        telemetry.addData("2. Velocity Ready", velocityReady + " (Err: " + (int) currentError + " < 20)");
        telemetry.addData("-> WILL SHOOT", isCanShooting(distance));
        telemetry.addData("Auto velocity", automatedVelocity);
        telemetry.addData("Wall pose", wallServo.getPosition());
        telemetry.addData("Master RPM", masterMotor.getVelocity());
        telemetry.addData("Slave RPM", slaveMotor.getVelocity());
        telemetry.addData("Fish Sensor", intakeModule.getState());
    }

    private boolean isCanShooting(double distance) {
        boolean turretAligned = Math.abs(bearing) < 8.0 && distance != -999999;
        return Math.abs(currentError) <= 35 && turretAligned;
    }

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double currentError = 0.0;
    private double pidOutput = 0.0;

    private void updatePID(double targetVelocity, DcMotorEx master, DcMotorEx slave) {
        double current_time = pidTimer.milliseconds();
        double currentVelocity = master.getVelocity();
        currentError = targetVelocity - currentVelocity;

        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) return;

        if (targetVelocity > 0) {
            if (currentError > gangBangError) {
                pidOutput = 1.0;
            } else {
                double fComponent = targetVelocity * RobotConstants.ShooterPid.f;
                double pComponent = RobotConstants.ShooterPid.p * currentError;
                double derivative = (currentError - lastError) / deltaTime;
                double dComponent = RobotConstants.ShooterPid.d * derivative;

                pidOutput = fComponent + pComponent + dComponent;
            }
        } else {
            pidOutput = 0;
        }

        pidOutput = Math.clamp(pidOutput, 0, 1.0);
        slave.setPower(pidOutput);
        master.setPower(pidOutput);

        lastError = currentError;
        lastTime = current_time;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Shooter PID", pidOutput);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("master", master.getVelocity());
        PanelsTelemetry.INSTANCE.getTelemetry().addData("target", targetVelocity);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("error", currentError);
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public void updateTarget() {
        if (useLogitech) {
            if (!cameraActive || camera.getDistance(aprilTagId) == -1)
                return;

            distance = camera.getDistance(aprilTagId);
            bearing = camera.getBearing(aprilTagId);
        } else if ((result = limelight3A.getLatestResult()) != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fid : fiducialResults) {
                if (fid.getFiducialId() == aprilTagId) {
                    distance = fid.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.METER).z;
                    bearing = result.getTx();
                }
            }
        } else {
            bearing = -999999;
            distance = -999999;
        }

        if (distance != -999999) {
            automatedServoPose = distToWallPos(distance);
            automatedVelocity = distToVelocityApprox(distance);
        }
    }

    private void resetPID() {
        lastError = 0;
        lastTime = 0;
        pidOutput = 0;
        pidTimer.reset();
    }

    public AprilTagsDetection getCamera() {
        return camera;
    }

    public double getBearing() {
        return bearing;
    }

    public double getDistance() {
        return distance;
    }

    private void updateWall(double pos) {
        wallServo.setPosition(pos);
    }

    private double servoToAngle(double servo) {
        return 20 * Math.PI / 180 + (servo * 45 * Math.PI / 180);
    }

    private double distToWallPos(double x) {
        return x > 0.94 ? 1 : 0.3;
    }

    private double distToVelocityPhysic(double dist, double launchAngle) {
        return Math.sqrt(9.81 * dist * dist / (dist * Math.sin(2 * launchAngle) - 2 * (1.38 - 0.237) * Math.cos(launchAngle) * Math.cos(launchAngle))) * 185 / 0.6;
    }

    private double distToVelocityApprox(double x) {
        return -79.0213 * x * x * x + 353.2320 * x * x - 189.8235 * x + 1430.7563;
    }
}