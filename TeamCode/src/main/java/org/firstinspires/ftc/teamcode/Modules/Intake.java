package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake {
    DcMotor motor;
    Gamepad g;
    int autoCycles = 0;
    boolean waiting = true;
    ElapsedTime timer = new ElapsedTime();

    public Intake(LinearOpMode lom) {
        motor = lom.hardwareMap.get(DcMotor.class, RobotConstants.IntakeMotor);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        g = lom.gamepad1;
    }

    public void TeleOp() {
        if (g.right_bumper)
            motor.setPower(1);
        else if (g.left_bumper)
            motor.setPower(-1);
        else
            motor.setPower(0);

    }

    public void ResetTimer() {
        timer.reset();
    }

    public void Autonom(double power, double workTime, double waitTime, int cycles) {
        if (autoCycles < cycles) {
            if (!waiting && workTime > timer.milliseconds()) {
                motor.setPower(power);
            } else if (!waiting && workTime <= timer.milliseconds()) {
                waiting = true;
                timer.reset();
            } else if (waiting && waitTime > timer.milliseconds()) {
                motor.setPower(0);
            } else if (waiting && waitTime <= timer.milliseconds()) {
                waiting = false;
                timer.reset();
                autoCycles++;
            }
        } else {
            motor.setPower(0);
        }
    }

}
