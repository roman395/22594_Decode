package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Camera.AprilTagsDetection;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Configurable
public class Shooter {
    public static double approxVelCof = 0.2303, approxWallCof = 0.00058, approxVelOf = 1535.3528, approxWallOf = 0.08927;
    DcMotorImplEx leftM, rightM, forwardEncoder, strafeEncoder;
    public static double velocity = 2000;
    public static double pos = 0;
    public AprilTagsDetection detect;
    int current_pos = 0;
    Servo rightS, leftS;
    Gamepad g;
    Telemetry t;
    public static PIDCoefficients pid = new PIDCoefficients(0.013, 0, 0.005);
    ElapsedTime timer = new ElapsedTime();
    private double current_velocity = 1800;
    private boolean isShooting = false, isFirstIntake = false, isRumble;
    ElapsedTime pidTimer = new ElapsedTime();
    ExposureControl control;
    public static boolean useCamera = true, usePhysic = false;
    double max_i = 0;

    public Shooter(LinearOpMode lom, Telemetry tel) {
        g = lom.gamepad1;
        t = tel;

        leftM = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ShootLeft);
        rightM = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ShootRight);
        rightS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoR);
        leftS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoL);
        leftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
        forwardEncoder = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ForwardEncoder);
        strafeEncoder = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.StrafeEncoder);

        leftS.setDirection(Servo.Direction.REVERSE);
        detect = new AprilTagsDetection(lom, t);
        getVisionPortal().resumeStreaming();
        getVisionPortal().resumeLiveView();
        PanelsCameraStream.INSTANCE.startStream(getVisionPortal(), 30);
        control = getVisionPortal().getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        control.setExposure(3, TimeUnit.MILLISECONDS);

    }

    private void Telemetry() {
        if (useCamera)
            t.addData("distance", detect.GetDistance(24));
        t.addData("right velocity", rightM.getVelocity());
        t.addData("left velocity", leftM.getVelocity());
        t.addData("time", current_time);
        t.addData("current velocity", current_velocity);
        t.addData("error", current_error);
        t.addData("changed velocity", output);
        t.addData("power", leftM.getPower());
        t.addData("wall", pos);
    }

    public void VelocityTests(double velocity, double pos, int id, boolean isCameraUse) {
        detect.Update();
        t.addData("distance", detect.GetDistance(id));
        if (isCameraUse) {
            PID(rightM, leftM, DistToVelocity(detect.GetDistance(id)));
            leftS.setPosition(DistToWallPose(detect.GetDistance(id)));
        } else
            PID(rightM, leftM, velocity);
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        t.addData("right velocity", rightM.getVelocity());
        t.addData("left velocity", leftM.getVelocity());
        t.addData("time", current_time);
        t.addData("current velocity", velocity);
        t.addData("error", current_error);
        t.addData("changed velocity", output);
        t.addData("power", leftM.getPower());
        t.addData("wall", pos);
    }

    public void TeleOp(Intake intake, int id) {
        //Testing(velocity, pos);
        if (g.startWasPressed())
            useCamera = !useCamera;

        detect.Update();
        if (detect.GetDistance(id) != -1 && useCamera) {
            if (!usePhysic) {
                current_velocity = DistToVelocity(detect.GetDistance(id));
                pos = DistToWallPose(detect.GetDistance(id));
            } else {
                pos = LaunchAngleToServo(DistToLaunchAngle(detect.GetElevation()));
                current_velocity = VelocityToTPS(DistToVelocityPhysic(detect.GetDistance(id) / 1000, DistToLaunchAngle(detect.GetElevation())));
                t.addData("Velocity Physic", current_velocity);
                t.addData("pos Physic", pos);
                t.addData("angle degree",Math.toDegrees(DistToLaunchAngle(detect.GetElevation())));
                t.addData("elevation", detect.GetElevation());
            }
            if (isShooting) {
                if (isRumble)
                    t.addLine("RumblingTime!");
                if (Math.abs(detect.GetBearing()) < 10 && Math.abs(detect.GetBearing()) > 7 && !isRumble) {
                    g.rumble(500);
                    isRumble = true;
                } else if (isRumble && !(Math.abs(detect.GetBearing()) < 10 && Math.abs(detect.GetBearing()) > 7))
                    isRumble = false;
            }
        } else if (!useCamera && !usePhysic)
            current_velocity = velocity;
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        if (isShooting) PID(rightM, leftM, current_velocity);
        if (g.rightBumperWasReleased()) timer.reset();
        if (g.rightBumperWasPressed() && !isFirstIntake) isFirstIntake = true;
        if (g.circle)
            isShooting = true;
        else if (g.square) {
            leftM.setPower(-0);
            rightM.setPower(-0);
            isShooting = false;
        }
        if (!isShooting && (g.right_bumper || timer.milliseconds() < 1500) && isFirstIntake) {
            leftM.setPower(-0.6);
            rightM.setPower(-0.6);
        }
        if (g.right_bumper && !isShooting)
            intake.ShooterEnable(1);
        else if (isShooting && g.right_bumper)
            intake.ShooterEnable(0.6);
        else if (g.left_bumper)
            intake.ShooterEnable(-0.6);
        else
            intake.ShooterEnable(0);

        if (!g.right_bumper && !isShooting && !g.triangle) {
            leftM.setPower(-0);
            rightM.setPower(-0);
        }

        if (g.triangle && !isShooting) {
            leftM.setPower(-0.5);
            rightM.setPower(-0.5);
        }
        if (!useCamera) {
            g.setLedColor(255, 0, 0, 1000);
            if (g.dpadLeftWasPressed())
                current_velocity -= 50;
            else if (g.dpadRightWasPressed())
                current_velocity += 50;
            if (g.dpadUpWasPressed())
                pos += 0.05;
            else if (g.dpadDownWasPressed())
                pos -= 0.05;
        } else {
            g.setLedColor(0, 255, 0, 1000);

            if (g.dpadLeftWasPressed())
                approxVelCof -= 0.01;
            else if (g.dpadRightWasPressed())
                approxVelCof += 0.01;
        }
        Telemetry();
    }

    public void ResetTimer() {
        timer.reset();
    }

    public void Update() {
        detect.Update();
    }

    public boolean Autonom(double time, int id) {
        if (detect.GetDistance(id) != -1) {
            current_velocity = DistToVelocity(detect.GetDistance(id));
            pos = DistToWallPose(detect.GetDistance(id));
        }
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        if (time > timer.milliseconds()) {
            PID(rightM, leftM, current_velocity);
        } else {
            rightM.setPower(0);
            leftM.setPower(0);
            return true;
        }
        return false;
    }

    double current_time = 0, current_error = 0, previous_time = 0, previous_error = 0, output, previous_output, i = 0, p, d;

    private void PID(DcMotorEx motorR, DcMotorEx motorL, double targetVelocity) {
        current_time = pidTimer.milliseconds();
        current_error = targetVelocity - motorL.getVelocity();
        p = pid.p * current_error;
        i += pid.i * (current_error * (current_time - previous_time));
        d = pid.d * (current_error - previous_error) / (current_time - previous_time);
        if (i > max_i) i = max_i;
        else if (i < -max_i) i = -max_i;
        output = p + i + d;
        if (targetVelocity != 0) {
            motorR.setPower(output);
            motorL.setPower(output);
        } else {
            motorR.setPower(0);
            motorL.setPower(0);
        }

        previous_error = current_error;
        previous_time = current_time;
    }

    private double DistToVelocity(double dist) {
        //if(dist < 2000)
        return dist * approxVelCof + approxVelOf;

    }

    private double DistToWallPose(double dist) {
        return Math.min(dist * approxWallCof - approxWallOf, 0.95);
    }

    public VisionPortal getVisionPortal() {
        return detect.getPortal();
    }

    private double DistToLaunchAngle(double elevation) {
        return Math.atan(Math.tan(Math.toRadians(elevation)));
    }

    private double LaunchAngleToServo(double angle) {
        return Math.toDegrees(angle) / 180 * 0.95;
    }

    private double DistToVelocityPhysic(double dist, double launchAngle) {
        return Math.sqrt(9.81 * dist / Math.sin(2 * launchAngle));
    }

    private double VelocityToTPS(double velocity) {
        return (28 * 28 / 22.0 * velocity) / (Math.PI * 0.072);
    }
}
