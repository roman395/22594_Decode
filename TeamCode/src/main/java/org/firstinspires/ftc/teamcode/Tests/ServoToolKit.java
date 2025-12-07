package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Configurable
public class ServoToolKit extends LinearOpMode {
    public static String name1 = "";
    public static boolean isReverse1 = false;
    public static String name2 = "";
    public static boolean isReverse2 = false;
    public static String name3 = "";
    public static boolean isReverse3 = false;

    public static double pos1 = 0;
    public static double pos2 = 0;
    public static double pos3 = 0;

    Servo servo1;
    Servo servo2;
    Servo servo3;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager tel = PanelsTelemetry.INSTANCE.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            servo1 = hardwareMap.tryGet(Servo.class, name1);
            servo2 = hardwareMap.tryGet(Servo.class, name2);
            servo3 = hardwareMap.tryGet(Servo.class, name3);
            if (servo1 != null) {
                if (isReverse1)
                    servo1.setDirection(Servo.Direction.REVERSE);
                else
                    servo1.setDirection(Servo.Direction.FORWARD);
                servo1.setPosition(pos1);
            } else
                tel.addLine("SERVO 1 NAME ERROR");
            if (servo2 != null) {
                if (isReverse2)
                    servo2.setDirection(Servo.Direction.REVERSE);
                else
                    servo2.setDirection(Servo.Direction.FORWARD);
                servo2.setPosition(pos2);
            } else
                tel.addLine("SERVO 2 NAME ERROR");
            if (servo3 != null) {
                if (isReverse3)
                    servo3.setDirection(Servo.Direction.REVERSE);
                else
                    servo3.setDirection(Servo.Direction.FORWARD);
                servo3.setPosition(pos3);
            } else
                tel.addLine("SERVO 3 NAME ERROR");
            tel.update();
        }
    }
//s1 - 0 - правый
    //s2 - 1 - левый реверс
}