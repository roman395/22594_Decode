package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class SensorsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel rev = hardwareMap.get(DigitalChannel.class,"rev");
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("distance", rev.getState());
            telemetry.update();
        }
    }
}
