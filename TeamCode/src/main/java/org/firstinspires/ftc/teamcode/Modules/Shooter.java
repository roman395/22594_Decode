package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Shooter
{
    private final DcMotor motor;

    public Shooter(LinearOpMode lom)
    {
        motor = lom.hardwareMap.get(DcMotor.class, RobotConstants.ShooterMotor);
    }

    public void TeleOp()
    {

    }
}
