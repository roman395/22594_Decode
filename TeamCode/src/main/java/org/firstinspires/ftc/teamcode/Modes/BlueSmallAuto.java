package org.firstinspires.ftc.teamcode.Modes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;

@Configurable

@Autonomous(name = "SUPERDOOPER")
public class BlueSmallAuto extends LinearOpMode {
    Mecanum drive;
    Shooter shooter;
    Intake intake;
    public static double drivePower = 1;
    public static double driveTime = 0;
    public static double shooterTime = 10;
    public static double shooterPower = 0.6;
    public static double wallPose = 0.6;
    public static double intakePower = 1;
    public static double intakeWaitTime = 2;
    public static double intakeWorkTime = 2;
    public static int intakeCycles = 3;

    boolean isTrainComplete, firstTrainComplete = false;
    boolean isShooterComplete, firstShooterCompletes = false;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTime *= 1000;
        shooterTime *= 1000;
        intakeWaitTime *= 1000;
        intakeWorkTime *= 1000;

        drive = new Mecanum(this);
        shooter = new Shooter(this);
        intake = new Intake(this);

        waitForStart();

        drive.ResetTimer();


        while (opModeIsActive()) {
            isTrainComplete = drive.Autonom(driveTime, drivePower);
            if(isTrainComplete && !firstTrainComplete){
                shooter.ResetTimer();
                intake.ResetTimer();
                firstTrainComplete = true;
            }
            if (isTrainComplete) {
                intake.Autonom(intakePower, intakeWorkTime, intakeWaitTime, intakeCycles);
                shooter.Autonom(shooterTime, shooterPower, wallPose);
            }

        }
    }
}
