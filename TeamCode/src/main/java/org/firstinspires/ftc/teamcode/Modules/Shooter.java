package org.firstinspires.ftc.teamcode.Modules;

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
import org.firstinspires.ftc.teamcode.RobotConstants;

@Configurable
public class Shooter {
    DcMotorImplEx leftM, rightM, forwardEncoder, strafeEncoder;
    public static double velocity = 0;
    public static double pos = 0;
    Position[] positions = {new Position(0.8, 1600), new Position(0.8, 0), new Position(0.8, 0), new Position(1, 0)};
    int current_pos = 0;
    Servo rightS, leftS;
    Gamepad g;
    Telemetry t;
    public static PIDCoefficients pid = new PIDCoefficients(0.004, 0, 0.01);
    ElapsedTime timer = new ElapsedTime();
    private double current_velocity = 0;
    private boolean isShooting = false, isFirstIntake = false;
    ElapsedTime pidTimer = new ElapsedTime();

    public Shooter(LinearOpMode lom) {
        g = lom.gamepad1;
        t = lom.telemetry;

        leftM = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ShootLeft);
        rightM = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ShootRight);
        rightS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoR);
        leftS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoL);
        leftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        forwardEncoder = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ForwardEncoder);
        strafeEncoder = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.StrafeEncoder);

        leftS.setDirection(Servo.Direction.REVERSE);

        leftS.setPosition(positions[current_pos].wallPosition);
        rightS.setPosition(positions[current_pos].wallPosition);
        current_velocity = positions[current_pos].velocity;
    }

    public void VelocityTests(double velocity, double pos) {
        PID(rightM, leftM, velocity);
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        t.addData("right velocity", rightM.getVelocity());
        t.addData("left velocity", leftM.getVelocity());
        t.addData("time", current_time);
        t.addData("error", current_error);
        t.addData("changed velocity", output);
        t.addData("power", rightM.getPower());

    }

    public void TeleOp() {
        Testing(velocity, pos);
        if(isShooting && !g.triangle && !g.right_bumper)
          PID(rightM, leftM, current_velocity);
        if (g.rightBumperWasReleased())
            timer.reset();
        if (g.rightBumperWasPressed() && !isFirstIntake)
            isFirstIntake = true;
        if (g.circle) {
            isShooting = true;
        } else if (g.square) {
            PID(rightM, leftM, 0);
            isShooting = false;
        } else if (!isShooting && (g.right_bumper || timer.milliseconds() < 1500) && isFirstIntake) {
            leftM.setPower(-0.4);
            rightM.setPower(-0.4);
        } else if (!g.right_bumper && isShooting)
            PID(rightM, leftM, current_velocity);
        else if (!g.right_bumper && !isShooting && !g.triangle)
            PID(rightM, leftM, 0);

        else if (g.triangle && !isShooting) {
            leftM.setPower(-0.5);
            rightM.setPower(-0.5);
        }
        else if (!g.triangle && !isShooting)
            PID(rightM, leftM, 0);


        t.addData("right velocity", rightM.getVelocity());
        t.addData("left velocity", leftM.getVelocity());
        t.addData("time", current_time);
        t.addData("error", current_error);
        t.addData("changed velocity", output);
        t.addData("power", rightM.getPower());
    }

    public void ResetTimer() {
        timer.reset();
    }

    public void ResetPIDTimer() {
        pidTimer.reset();
    }

    public boolean Autonom(double time, double power, double wallPose) {
        leftS.setPosition(wallPose);
        rightS.setPosition(wallPose);
        if (time > timer.milliseconds()) {
            rightM.setPower(power);
            leftM.setPower(power);
        } else {
            rightM.setPower(0);
            leftM.setPower(0);
            return true;
        }
        return false;
    }

    double current_time = 0, current_error = 0, previous_time = 0, previous_error = 0, output, previous_output;

    private void PID(DcMotorEx motorR, DcMotorEx motorL, double targetVelocity) {
        current_time = pidTimer.milliseconds();
        current_error = targetVelocity - motorL.getVelocity();

        output = pid.p * current_error + pid.d * (current_error - previous_error) / (current_time - previous_time);

        motorR.setPower(output);
        motorL.setPower(output);
        if (output != 0)
            previous_output = output;
        previous_error = current_error;
        previous_time = current_time;
    }
    public void Testing(double velocity, double pos){
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        current_velocity = velocity;
    }
    public void PositionsListing() {
        if (g.dpadUpWasPressed() && current_pos + 1 < positions.length) {
            current_pos++;
            leftS.setPosition(positions[current_pos].wallPosition);
            rightS.setPosition(positions[current_pos].wallPosition);
            current_velocity = positions[current_pos].velocity;
        } else if (g.dpadDownWasPressed() && current_pos - 1 >= 0) {
            current_pos--;
            leftS.setPosition(positions[current_pos].wallPosition);
            rightS.setPosition(positions[current_pos].wallPosition);
            current_velocity = positions[current_pos].velocity;
        }
    }
}

//wall position 1
//power 0.8 near, power 1 far