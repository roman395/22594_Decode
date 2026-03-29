package org.firstinspires.ftc.teamcode.Modes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.GlobalStorage;
import org.firstinspires.ftc.teamcode.Modules.Turret;

@TeleOp
public class ResetTurret extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Turret tur = new Turret(this, new Pose(0,0,0));
        tur.resetStartPose();
        waitForStart();
        tur.saveData();
        GlobalStorage.lastPose = new Pose(112,112,Math.toRadians(90));
    }
}
