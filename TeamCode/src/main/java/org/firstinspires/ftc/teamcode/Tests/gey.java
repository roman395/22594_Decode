package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;

import java.util.concurrent.TimeUnit;

@TeleOp
public class gey extends LinearOpMode {
    Telemetry tel;
    ElapsedTime inactiveTimer = new ElapsedTime();
    Turret t;
    @Override
    public void runOpMode() throws InterruptedException {
        //t = new Turret(this);
        //Shooter sh = new Shooter(this, 23);
        //tel = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            //if(inactiveTimer.milliseconds() > 2000) {
              //  tur.AutoAimingOnError(sh.getBearing());
               // telemetry.addData("current angle", tur.GetCurrentPosition());
           // }
           // sh.updateTarget();
            //telemetry.addData("bearing", sh.getBearing());
            //telemetry.addData("distance", sh.getDistance());
            telemetry.addData("servo 1 output",1);
            telemetry.update();
//            tel.update();
        }
    }
}
