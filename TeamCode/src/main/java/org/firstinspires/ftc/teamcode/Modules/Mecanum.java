package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Mecanum {
    private final DcMotor FL, FR, RL, RR;
    private final Gamepad gamepad;
    ElapsedTime timer = new ElapsedTime();

    public Mecanum(LinearOpMode lom) {
        FL = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumFL);
        FR = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumFR);
        RL = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumRL);
        RR = lom.hardwareMap.get(DcMotor.class, RobotConstants.MecanumRR);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);
        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        gamepad = lom.gamepad1;
    }

    public void TeleOp() {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double slower = gamepad.cross ? 0.25 : 0.75;
        double rx = (gamepad.right_trigger - gamepad.left_trigger) * slower;
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
    }

    public void ResetTimer() {
        timer.reset();
    }

    public boolean Autonom(double time, double power) {
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

    public boolean Autonom(double time, double flPower, double frPower, double rlPower, double rrPower) {
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
}
