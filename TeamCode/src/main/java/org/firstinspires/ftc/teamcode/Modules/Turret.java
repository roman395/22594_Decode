package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Turret {
    private final Servo servo1, servo2;

    public Turret(LinearOpMode lom) {
        servo1 = lom.hardwareMap.get(Servo.class, RobotConstants.TurretServo1);
        servo2 = lom.hardwareMap.get(Servo.class, RobotConstants.TurretServo2);
    }

    public void TeleOp() {

    }
}
