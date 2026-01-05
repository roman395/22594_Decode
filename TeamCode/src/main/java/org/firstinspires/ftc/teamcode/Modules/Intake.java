package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake {
    DcMotor motor;
    Gamepad g;
    Gamepad gTest;
    int autoCycles = 0;
    boolean waiting = true;
    ElapsedTime timer = new ElapsedTime(), rollTimer = new ElapsedTime();

    public Intake(LinearOpMode lom) {
        motor = lom.hardwareMap.get(DcMotor.class, RobotConstants.IntakeMotor);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        g = lom.gamepad1;
        gTest = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(g);
    }

    public void TeleOp(double power) {
        if (g.right_bumper)
            motor.setPower(power);
        else if (g.left_bumper)
            motor.setPower(-power);
        else
            motor.setPower(0);

    }

    public void ShooterTest(double power) {
        motor.setPower(power);

    }

    public void ShooterEnable(double power) {
        motor.setPower(power);
    }

    public void ResetTimer() {
        timer.reset();
        rollTimer.reset();
    }

    public void Autonom(double power, double workTime, double waitTime, int cycles) {
        if(rollTimer.milliseconds() < 500)
            motor.setPower(-0.6);
        else if (autoCycles <= cycles) {
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
    public void Autonom(double power){
        motor.setPower(power);
    }

}
