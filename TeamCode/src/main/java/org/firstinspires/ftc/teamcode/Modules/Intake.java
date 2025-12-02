package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake {
    DcMotor motor;
    Gamepad g;

    public Intake(LinearOpMode lom) {
        motor = lom.hardwareMap.get(DcMotor.class, RobotConstants.IntakeMotor);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        g = lom.gamepad1;
    }

    public void TeleOp() {
        if (g.right_bumper)
            motor.setPower(1);
        else if(g.left_bumper)
            motor.setPower(-1);
        else
            motor.setPower(0);

    }
}
