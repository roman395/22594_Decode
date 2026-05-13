package org.firstinspires.ftc.teamcode.Deprecated.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class eee extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            m1.setPower(0.1);
            telemetry.addData("pose", m1.getVelocity());
            telemetry.update();
        }
    }
    /*
    Expantion
    tservo1: 5
    analog:1
    feeder: 2
    RLM:0
    FLM:1
    intake:3

    Control
    strafe:3
    forward:2
    RRM:3
    FRM:2
    shootL:0
    shootR:1
    tservo2:0
    anal:1
    wall:1
     */
}
