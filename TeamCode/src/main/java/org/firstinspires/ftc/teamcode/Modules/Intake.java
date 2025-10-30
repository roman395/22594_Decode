package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake
{
    private final DcMotor motor;

    public Intake(LinearOpMode lom)
    {
        motor = lom.hardwareMap.get(DcMotor.class, RobotConstants.IntakeMotor);
    }

    public void TeleOp()
    {

    }
}
