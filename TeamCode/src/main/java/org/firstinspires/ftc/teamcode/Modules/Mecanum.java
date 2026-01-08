package org.firstinspires.ftc.teamcode.Modules;

import android.webkit.ConsoleMessage;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

@Configurable
public class Mecanum {
    private final DcMotor FL, FR, RL, RR;
    private final Gamepad gamepad;
    ElapsedTime timer = new ElapsedTime();
    private boolean isAim;
    public static double targetAngle = 0, cof = 0;
    public static PIDFCoefficients pidf = new PIDFCoefficients(0.03,0,0,0);
    double rx = 0;
    ElapsedTime pidTimer = new ElapsedTime();
    public Mecanum(LinearOpMode lom) {
        FL = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumFL);
        FR = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumFR);
        RL = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumRL);
        RR = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumRR);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);
        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        gamepad = lom.gamepad1;
        pidTimer.reset();
    }

    public void TeleOp(double angle, Telemetry t) {
        if (gamepad.crossWasPressed())
            isAim = !isAim;
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        if (!isAim || angle == -999 || Math.abs(gamepad.right_trigger - gamepad.left_trigger) > 0.1)
            rx = (gamepad.right_trigger - gamepad.left_trigger) * 0.9;
        else {
            PID(angle);
            t.addData("angle", angle);
        }
        //double rx = gamepad.right_stick_x* slower;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double rlPower = (y - x + rx) / denominator;
        double frPower = (y - x - rx) / denominator;
        double rrPower = (y + x - rx) / denominator;

        FL.setPower(flPower);
        FR.setPower(frPower);
        RL.setPower(rlPower);
        RR.setPower(rrPower);
        t.addData("IsAim", isAim);
    }

    public void ResetTimer() {
        timer.reset();
    }

    public ElapsedTime GetTimer() {
        return timer;
    }

    public boolean ForwardMove(double time, double power) {
        if (time > timer.milliseconds()) {
            FL.setPower(power);
            FR.setPower(power);
            RL.setPower(power);
            RR.setPower(power);
        } else {
            FL.setPower(0);
            FR.setPower(0);
            RL.setPower(0);
            RR.setPower(0);
            return true;
        }
        return false;
    }

    public boolean StrafeMove(double time, double flPower, double frPower, double rlPower, double rrPower) {
        if (time > timer.milliseconds()) {
            FL.setPower(flPower);
            FR.setPower(frPower);
            RL.setPower(rlPower);
            RR.setPower(rrPower);
        } else {
            FL.setPower(0);
            FR.setPower(0);
            RL.setPower(0);
            RR.setPower(0);
            return true;
        }
        return false;
    }
    double current_time, current_error, p, i, d, f, previous_error = 0, previous_time = 0;
    private void PID(double angle){
        current_time = pidTimer.milliseconds();
        current_error = targetAngle-angle;
        p = pidf.p * current_error;
        f = pidf.f;
        d = pidf.d * (current_error - previous_error) / (current_time - previous_time);
        rx = p + f + d;
    }
}
