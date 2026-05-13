package org.firstinspires.ftc.teamcode.Deprecated.Modules;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
public class Mecanum {
    Follower follower;
    Gamepad g;
    IMU imu;
    ElapsedTime timer = new ElapsedTime();

    public Mecanum(LinearOpMode lom) {
        follower = Constants.createFollower(lom.hardwareMap);
        follower.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        follower.update();
        follower.startTeleOpDrive();
        g = lom.gamepad1;
        follower.startTeleOpDrive();
        imu = lom.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    }

    public void teleOp() {
        follower.update();
        follower.setTeleOpDrive(-g.left_stick_y, -g.left_stick_x, -(g.right_trigger - g.left_trigger) * 0.5, true);
        if (g.optionsWasPressed())
            imu.resetYaw();

    }
    public void loadStartPose(){
        follower.setStartingPose(GlobalStorage.lastPose);
    }
    public Pose getPose(){return follower.getPose();}


    public void ResetTimer() {
        timer.reset();
    }

    public void ForwardMove(int timeMillis, double power) {
        if (timer.milliseconds() < timeMillis) {
            follower.setTeleOpDrive(power, 0, 0);
            follower.startTeleOpDrive();
        } else {

        }

    }

}
