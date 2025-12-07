package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .forwardZeroPowerAcceleration(-33)
            .lateralZeroPowerAcceleration(-65)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0,0))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
            //.headingPIDFCoefficients(new PIDFCoefficients(0.67,0.001,0.01,0.02))
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
            .mass(10);
    public static TwoWheelConstants TWC = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(RobotConstants.ForwardEncoder)
            .strafeEncoder_HardwareMapName(RobotConstants.StrafeEncoder)
            .forwardPodY(1)
            .strafePodX(-6.22)
            .forwardTicksToInches(0.0021)
            .strafeTicksToInches(-0.0021)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
    public static MecanumConstants train = new MecanumConstants()
            .maxPower(1)
            .xVelocity(76.7)
            .yVelocity(60)
            .rightFrontMotorName(RobotConstants.MecanumFR)
            .leftFrontMotorName(RobotConstants.MecanumFL)
            .leftRearMotorName(RobotConstants.MecanumRL)
            .rightRearMotorName(RobotConstants.MecanumRR)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .twoWheelLocalizer(TWC)
                .mecanumDrivetrain(train)
                .build();
    }
}