package org.firstinspires.ftc.teamcode.Modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Barrel;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.ParkingLift;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;

@TeleOp
public class Teleop extends LinearOpMode
{
    Mecanum mecanum;
    Intake intake;
    Barrel barrel;
    Shooter shooter;
    Turret turret;
    ParkingLift lift;

    @Override
    public void runOpMode() throws InterruptedException
    {
        mecanum = new Mecanum(this);
        intake = new Intake(this);
        barrel = new Barrel(this);
        shooter = new Shooter(this);
        turret = new Turret(this);
        lift = new ParkingLift(this);

        waitForStart();

        while (opModeIsActive())
        {
            mecanum.TeleOp();
            intake.TeleOp();
            barrel.TeleOp();
            shooter.TeleOp();
            turret.TeleOp();
            lift.TeleOp();
        }
    }
}
