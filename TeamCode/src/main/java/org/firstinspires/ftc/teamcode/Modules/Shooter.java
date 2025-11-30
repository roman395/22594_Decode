package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

@Configurable
public class Shooter {
    DcMotor leftM;
    DcMotor rightM;
    Servo rightS;
    Servo leftS;
    double power = 1;
    Gamepad g;
    Telemetry t;
    double pos = 0;
    boolean wasPressed = false;

    public Shooter(LinearOpMode lom) {
        g = lom.gamepad1;
        t = lom.telemetry;
        leftM = lom.hardwareMap.get(DcMotor.class, RobotConstants.ShootLeft);
        rightM = lom.hardwareMap.get(DcMotor.class, RobotConstants.ShootRight);
        rightS = lom.hardwareMap.tryGet(Servo.class, RobotConstants.ShootServoR);
        leftS = lom.hardwareMap.tryGet(Servo.class, RobotConstants.ShootServoL);
        leftS.setDirection(Servo.Direction.REVERSE);
        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void TeleOp() {
        if (g.circle) {
            rightM.setPower(power);
            leftM.setPower(power);
        } else if (g.square) {
            rightM.setPower(0);
            leftM.setPower(0);
        } else if (g.triangle) {
            rightM.setPower(-0.4);
            leftM.setPower(-0.4);

            wasPressed = true;
        } else if (!g.triangle && wasPressed) {
            wasPressed = false;
            rightM.setPower(0);
            leftM.setPower(0);
        }
        if (g.dpadRightWasReleased()) {
            power += 0.05;
            rightM.setPower(power);
            leftM.setPower(power);
        } else if (g.dpadLeftWasReleased()) {
            rightM.setPower(power);
            leftM.setPower(power);

            power -= 0.05;
        }
        if (g.dpadDownWasPressed()) {
            pos -= 0.1;
        } else if (g.dpadUpWasReleased()) {
            pos += 0.1;
        }
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        t.addData("Shooter power", power);
    }
}
