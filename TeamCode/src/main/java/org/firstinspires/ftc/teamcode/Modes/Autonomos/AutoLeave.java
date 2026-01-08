package org.firstinspires.ftc.teamcode.Modes.Autonomos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Mecanum;
@Autonomous
public class AutoLeave extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive = new Mecanum(this);
        waitForStart();
        drive.ResetTimer();
        while (opModeIsActive())
            drive.ForwardMove(500,1);
    }
}
