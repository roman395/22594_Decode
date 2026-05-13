package org.firstinspires.ftc.teamcode.Deprecated.Modules;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Deprecated.RobotConstants;

@Configurable
public class Turret {
    private final CRServo s1;
    private final CRServo s2;
    private final AnalogInput s1En;
    private final AnalogInput s2En;
    private final Gamepad g;
    public static double maxAngle = 335, maxIntegral = 0.3, minAngle = -20, cameraMultiply = 1, servoRange = 370, imuErrorCof = 0.3;
    private final boolean isCloseToBreake = false;
    public static double breakPose = 3.2;
    public static double centerPose = 2.263;
    public static double inputMultiply = 0.5;
    private final boolean goStart = false;
    private boolean iCanSee = false;

    // Stability and Calibration Variables
    public static double headingCorrection = 0;
    public static double stabilityDelay = 500;

    private static final double MAX_VOLTAGE = 3.3;
    private static final double VOLTAGE_TO_ANGLE = servoRange / MAX_VOLTAGE;


    private final Telemetry t;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime seenTimer = new ElapsedTime();
    private double lastTime, countOfFullTurn = 0, integral = 0, pidOutput, lastError = 0;
    private double lastVoltage = 0;
    private double startPose = 0;
    private double targetPose = 0;
    Pose goalPose;
    Telemetry telemetry;
    private boolean isLocked = false;


    public Turret(LinearOpMode lom, Pose goalPose) {
        s1 = lom.hardwareMap.get(CRServo.class, RobotConstants.TurretServo1);
        s2 = lom.hardwareMap.get(CRServo.class, RobotConstants.TurretServo2);
        this.goalPose = goalPose;
        s1En = lom.hardwareMap.get(AnalogInput.class, RobotConstants.TurretServoEncoder1);
        s2En = lom.hardwareMap.get(AnalogInput.class, RobotConstants.TurretServoEncoder2);

        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMPORTANT: Axon encoder initialization
        s1.setPower(1);
        s2.setPower(1);
        s1.setPower(0);
        s2.setPower(0);

        g = lom.gamepad1;
        t = lom.telemetry;
        loadData();
        lastVoltage = s1En.getVoltage();
        startPose = lastVoltage;
        telemetry = lom.telemetry;
    }

    public void TeleOp() {
        double currentPos = GetCurrentPosition();

        if (g.right_stick_x < -0.05 && currentPos > minAngle) {
            s1.setPower(g.right_stick_x * inputMultiply);
            s2.setPower(g.right_stick_x * inputMultiply);
        } else if (g.right_stick_x > 0.05 && currentPos < maxAngle) {
            s1.setPower(g.right_stick_x * inputMultiply);
            s2.setPower(g.right_stick_x * inputMultiply);
        } else {
            s1.setPower(0);
            s2.setPower(0);
        }
        t.addData("turret angle:", currentPos);
        t.addData("heading correction:", headingCorrection);
    }

    public void AutoAimingOnTarget(double bearing) {
        double currentServoAngle = GetCurrentPosition();
        double targetServoAngle = Math.clamp(bearing * cameraMultiply, minAngle, maxAngle);
        double power = PIDOnTarget(targetServoAngle, currentServoAngle);
        s1.setPower(power);
        s2.setPower(power);
    }

    public void AutoAimingOnError(double error, Pose current, Shooter.STATE st) {
        double currentServoAngle = GetCurrentPosition();
        if(g.backWasPressed())
            isLocked =!isLocked;
        if(!isLocked) {
            if (error != -999999 && Math.abs(error) < 90) {
                seenTimer.reset();
                iCanSee = true;

                // Heading correction logic (standard 360 circle)
                double fieldAngleDeg = Math.toDegrees(Math.atan2(goalPose.getY() - current.getY(), goalPose.getX() - current.getX()));
                double relativeGoalAngleDeg = currentServoAngle + error;
                double actualHeadingDeg = fieldAngleDeg + relativeGoalAngleDeg;
                double currentHeadingDeg = Math.toDegrees(current.getHeading());

                headingCorrection = actualHeadingDeg - currentHeadingDeg;
                while (headingCorrection > 180) headingCorrection -= 360;
                while (headingCorrection <= -180) headingCorrection += 360;

                // Target calculation
                double targetAngle = currentServoAngle + error;

                // Intelligent Range-Aware Wrapping
                while (targetAngle - currentServoAngle > 180) targetAngle -= 360;
                while (targetAngle - currentServoAngle <= -180) targetAngle += 360;

                // Check if jumping 360 degrees recovers a valid position
                if (targetAngle < minAngle && targetAngle + 360 <= maxAngle) targetAngle += 360;
                else if (targetAngle > maxAngle && targetAngle - 360 >= minAngle)
                    targetAngle -= 360;

                if (targetAngle >= minAngle && targetAngle <= maxAngle) {
                    double power = PIDOnError(targetAngle - currentServoAngle, currentServoAngle);
                    s1.setPower(power);
                    s2.setPower(power);
                } else {
                    s1.setPower(0);
                    s2.setPower(0);
                }
            } else {
                if (!(st == Shooter.STATE.SHOOTING && g.right_bumper)) {
                    if (seenTimer.milliseconds() > stabilityDelay || !iCanSee) {
                        iCanSee = false;
                        aimingOnOdo(goalPose, current);
                    }
                } else {
                    s1.setPower(0);
                    s2.setPower(0);
                }
            }
        }
    }

    private double PIDOnTarget(double target, double current) {
        double current_time = pidTimer.milliseconds();
        double error = target - current;
        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) return 0;

        double pComponent = RobotConstants.TurretPid.p * error;
        integral = Math.clamp(integral + (error * deltaTime), -maxIntegral, maxIntegral);
        double iComponent = integral * RobotConstants.TurretPid.i;
        double derivative = (error - lastError) / deltaTime;
        double dComponent = RobotConstants.TurretPid.d * derivative;
        double fComponent = (Math.abs(error) > 0.5) ? Math.signum(error) * RobotConstants.TurretPid.f : 0;

        lastError = error;
        lastTime = current_time;
        return pComponent + dComponent + iComponent + fComponent;
    }

    private double PIDOnError(double error, double current) {
        double current_time = pidTimer.milliseconds();
        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) return 0;

        double pComponent = RobotConstants.TurretPid.p * error;
        integral = Math.clamp(integral + (error * deltaTime), -maxIntegral, maxIntegral);
        double iComponent = integral * RobotConstants.TurretPid.i;
        double derivative = (error - lastError) / deltaTime;
        double dComponent = RobotConstants.TurretPid.d * derivative;
        double fComponent = (Math.abs(error) > 0.5) ? Math.signum(error) * RobotConstants.TurretPid.f : 0;

        lastError = error;
        lastTime = current_time;
        return pComponent + dComponent + iComponent + fComponent;
    }

    public double GetCurrentPosition() {
        double currentVoltage = s1En.getVoltage();
        currentVoltage = Math.min(MAX_VOLTAGE, Math.max(0, currentVoltage));
        double voltageChange = currentVoltage - lastVoltage;

        if (voltageChange > MAX_VOLTAGE / 2) countOfFullTurn--;
        else if (voltageChange < -MAX_VOLTAGE / 2) countOfFullTurn++;

        lastVoltage = currentVoltage;
        return (currentVoltage - centerPose) * VOLTAGE_TO_ANGLE + countOfFullTurn * servoRange;
    }

    public void resetStartPose() {
        centerPose = s1En.getVoltage();
        countOfFullTurn = 0;
        lastVoltage = centerPose;
    }

    public void advancedTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret Angle", GetCurrentPosition());
        telemetry.addData("Heading Correction", headingCorrection);
    }

    public void updateTurret(double error) {
        if (Math.abs(error) > 200 && !goStart) {
            s1.setPower(PIDOnTarget(targetPose, GetCurrentPosition()));
            s2.setPower(PIDOnTarget(targetPose, GetCurrentPosition()));
        } else {
            s1.setPower(PIDOnError(error, GetCurrentPosition()));
            s2.setPower(PIDOnError(error, GetCurrentPosition()));
        }
    }

    public void setTargetPose(double targetPose) {
        this.targetPose = targetPose;
    }

    public void saveData() {
        GlobalStorage.lastTurretCenterPose = centerPose;
        GlobalStorage.lastTurretFullTurns = countOfFullTurn;
        GlobalStorage.lastDriftOffset = headingCorrection;
    }

    public void loadData() {
        centerPose = GlobalStorage.lastTurretCenterPose;
        countOfFullTurn = GlobalStorage.lastTurretFullTurns;
        headingCorrection = GlobalStorage.lastDriftOffset;
    }

    public void aimingOnOdo(Pose goal, Pose current) {
        double currentServoAngle = GetCurrentPosition();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(goal.getY() - current.getY(), goal.getX() - current.getX()));
        double correctedHeading = Math.toDegrees(current.getHeading()) + headingCorrection;
        double targetAngle = correctedHeading - fieldAngleDeg;

        // Intelligent Range-Aware Wrapping
        while (targetAngle - currentServoAngle > 180) targetAngle -= 360;
        while (targetAngle - currentServoAngle <= -180) targetAngle += 360;

        // Check if jumping 360 degrees recovers a valid position
        if (targetAngle < minAngle && targetAngle + 360 <= maxAngle) targetAngle += 360;
        else if (targetAngle > maxAngle && targetAngle - 360 >= minAngle) targetAngle -= 360;

        targetAngle = Math.clamp(targetAngle, minAngle, maxAngle);
        double power = PIDOnTarget(targetAngle, currentServoAngle);
        s1.setPower(power);
        s2.setPower(power);
    }
}
