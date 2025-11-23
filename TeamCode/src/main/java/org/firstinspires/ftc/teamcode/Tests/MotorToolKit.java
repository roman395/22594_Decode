package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Configurable
public class MotorToolKit extends LinearOpMode {
    public static String name1 = "";
    public static String name2 = "";
    public static String name3 = "";

    public static double power1 = 0;
    public static double power2 = 0;
    public static double power3 = 0;

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager tel = PanelsTelemetry.INSTANCE.getTelemetry();
        waitForStart();
        while (opModeIsActive()){
            motor1 = hardwareMap.tryGet(DcMotor.class, name1);
            motor2 = hardwareMap.tryGet(DcMotor.class, name2);
            motor3 = hardwareMap.tryGet(DcMotor.class, name3);
            if (motor1 != null)
                motor1.setPower(power1);
            else
                tel.addLine("MOTOR 1 NAME ERROR");
            if (motor2 != null)
                motor2.setPower(power2);
            else
                tel.addLine("MOTOR 2 NAME ERROR");
            if (motor3 != null)
                motor3.setPower(power3);
            else
                tel.addLine("MOTOR 3 NAME ERROR");
            tel.update();
        }
    }

}