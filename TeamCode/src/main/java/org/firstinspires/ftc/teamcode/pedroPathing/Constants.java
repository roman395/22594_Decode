package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-31.6)
            .lateralZeroPowerAcceleration(-51.98)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.01,0.04))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5,0.001,0.005,0.03))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.08,0.01))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0,0,0,0,0))
            .mass(10);
    public static TwoWheelConstants TWC = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(RobotConstants.ForwardEncoder)
            .strafeEncoder_HardwareMapName(RobotConstants.StrafeEncoder)
            .forwardPodY(1)
            .strafePodX(-6.22)
            .forwardTicksToInches(0.002)
            .strafeTicksToInches(-0.0019)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
    public static MecanumConstants train = new MecanumConstants()
            .maxPower(1)
            .xVelocity(72)
            .yVelocity(53)
            .rightFrontMotorName(RobotConstants.MecanumFR)
            .leftFrontMotorName(RobotConstants.MecanumFL)
            .leftRearMotorName(RobotConstants.MecanumRL)
            .rightRearMotorName(RobotConstants.MecanumRR)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
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