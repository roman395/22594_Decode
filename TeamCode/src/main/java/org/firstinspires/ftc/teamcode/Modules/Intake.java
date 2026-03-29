package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Intake {
    private final double intakePower = 1;
    private final double feederPower = -1;
    private final DcMotor intakeMotor, feederMotor;
    private final Rev2mDistanceSensor fish;
    public static double normalDistance = 185;

    public Intake(LinearOpMode linearOpMode) {
        intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, RobotConstants.IntakeMotor);
        feederMotor = linearOpMode.hardwareMap.get(DcMotor.class, RobotConstants.FeederMotor);
        fish = linearOpMode.hardwareMap.get(Rev2mDistanceSensor.class, RobotConstants.RevDistanceSensor);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void setFeederPower(double power) {
        feederMotor.setPower(power);
    }

    public void grabbing() {
        intakeMotor.setPower(intakePower);
    }

    public void feeding() {
        feederMotor.setPower(feederPower);
    }

    public void stopGrabbing() {
        intakeMotor.setPower(0);
    }

    public void stopFeeding() {
        feederMotor.setPower(0);
    }

    public boolean getState() {
        return fish.getDistance(DistanceUnit.MM) >= normalDistance - 10;
    }

    public void stopAll() {
        feederMotor.setPower(0);
        intakeMotor.setPower(0);
    }

    public void enableAll() {
        feederMotor.setPower(feederPower);
        intakeMotor.setPower(intakePower);
    }

    public void intakeControl(Gamepad gamepad) {
        if (gamepad.right_bumper)
            grabbing();
        else if (gamepad.left_bumper)
            setIntakePower(-intakePower);
        else
            stopGrabbing();
    }

    public void feedingControl(Gamepad gamepad) {
        if (gamepad.right_bumper)
            feeding();

        else if (gamepad.left_bumper)
            setFeederPower(-feederPower);
        else
            stopFeeding();
    }

    public void controlUntilShooting(Gamepad gamepad, boolean notUsingFish) {
        if (!notUsingFish) {
            if (gamepad.right_bumper && getState()) {
                grabbing();
                feeding();
            } else if (gamepad.right_bumper) {
                grabbing();
                stopFeeding();
            } else if (gamepad.left_bumper) {
                setIntakePower(-intakePower);
                setFeederPower(-feederPower);
            } else {
                stopGrabbing();
                stopFeeding();
            }
        } else {

            if (gamepad.right_bumper) {
                grabbing();
            } else if (gamepad.left_bumper) {
                grabbing();
                feeding();
            } else {
                stopGrabbing();
                stopFeeding();
            }

        }
    }

    public void grabbingAutonomous() {
        if (getState()) {
            grabbing();
            feeding();
        } else {
            grabbing();
            stopFeeding();
        }
    }
    public double getDistance(){
        return fish.getDistance(DistanceUnit.MM);
    }

}

