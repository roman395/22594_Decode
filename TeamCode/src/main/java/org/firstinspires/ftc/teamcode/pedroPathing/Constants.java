package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10);
    public static TwoWheelConstants TWC = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(RobotConstants.ForwardEncoder)
            .strafeEncoder_HardwareMapName(RobotConstants.StrafeEncoder)
            .forwardPodY(1)
            .strafePodX(1)
            //.forwardTicksToInches(1)
            //.strafeTicksToInches(1)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                            RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                    )
            );
    public static MecanumConstants train = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotConstants.MecanumFR)
            .leftFrontMotorName(RobotConstants.MecanumFL)
            .leftRearMotorName(RobotConstants.MecanumRL)
            .rightRearMotorName(RobotConstants.MecanumRR)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(TWC)
                .mecanumDrivetrain(train)
                .build();
    }
}