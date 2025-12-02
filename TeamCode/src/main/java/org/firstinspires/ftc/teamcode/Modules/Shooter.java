package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

@Configurable
public class Shooter{
    //wall position, power
    double[][] preset = {{1,1}, {1,0.8}};
    DcMotorEx leftM, rightM, forwardEncoder, strafeEncoder;
    Servo rightS, leftS;
    double power = 0.8, last = 0.8, pos = 1, strafeMultiplicator = 0, forwardMultiplicator = 0, linearMultiplicator;
    Gamepad g;
    Telemetry t;
    Pose positionNow;
    public static PIDCoefficients pid = new PIDCoefficients(0, 0, 0);
    private boolean isShooting = false;
    ElapsedTime timer = new ElapsedTime();

    public Shooter(LinearOpMode lom) {
        g = lom.gamepad1;
        t = lom.telemetry;

        leftM = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootLeft);
        rightM = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootRight);
        rightS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoR);
        leftS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoL);
        leftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forwardEncoder = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ForwardEncoder);
        strafeEncoder = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.StrafeEncoder);

        leftS.setDirection(Servo.Direction.REVERSE);
        //rightM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void TeleOp() {
        if (g.rightBumperWasReleased())
            timer.reset();
        if (g.circle) {
            rightM.setPower(power);
            leftM.setPower(power);
            isShooting = true;
        } else if (g.square) {
            rightM.setPower(0);
            leftM.setPower(0);
            isShooting = false;
        } else if (!isShooting && (g.right_bumper || timer.milliseconds() < 1500)) {
            rightM.setPower(-0.2);
            leftM.setPower(-0.2);
        } else if (!g.right_bumper && isShooting) {
            rightM.setPower(power);
            leftM.setPower(power);
        } else if (!g.right_bumper && !isShooting && !g.triangle) {
            rightM.setPower(0);
            leftM.setPower(0);
        } else if (g.triangle && !isShooting) {
            rightM.setPower(-0.5);
            leftM.setPower(-0.5);
        } else if (!g.triangle && !isShooting) {
            rightM.setPower(0);
            leftM.setPower(0);
        }
        if (g.dpadRightWasReleased()) {
            power += 0.05;
        } else if (g.dpadLeftWasReleased()) {
            power -= 0.05;
        }
        ManualWallTranslation();
        t.addData("Shooter power", power);

        t.addData("left shooter voltage", leftM.getVelocity());
        t.addData("right shooter voltage", rightM.getVelocity());
        t.addData("wall position", pos);
    }

    private void ManualWallTranslation() {
        if (g.dpadDownWasPressed()) {
            pos -= 0.1;
        } else if (g.dpadUpWasReleased()) {
            pos += 0.1;
        }
        leftS.setPosition(pos);
        rightS.setPosition(pos);
    }

    private void AutoWallTranslation(double distance) {
        double turn = distance * linearMultiplicator;
        leftS.setPosition(turn);
        rightS.setPosition(turn);
    }

    private double CalculateDistance(Pose startPose, Pose goal) {
        double x, y;
        x = strafeEncoder.getCurrentPosition() * strafeMultiplicator;
        y = forwardEncoder.getCurrentPosition() * forwardMultiplicator;

        return new Pose(x + startPose.getX(), y + startPose.getY()).distanceFrom(goal);
    }
    //private void SetPreset

    private void PID() {

    }
}

//wall position 1
//power 0.8 near, power 1 far