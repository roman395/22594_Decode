package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Turret {
    private CRServo s1, s2;
    private AnalogInput s1En, s2En;
    private Gamepad g;
    public static double maxAngle = 0, minAngle = 0, cameraMultiply = 1, servoRange = 355;//TODO don`t forget about gear ratio
    public static double centerPose = 0, leftPose = 0, rightPose = 0;
    private Telemetry t;
    private ElapsedTime pidTimer = new ElapsedTime();
    public static PIDCoefficients pid = new PIDCoefficients(0, 0, 0);
    private double lastTime, countOfFullTurn = 0, pidOutput, lastError = 0, lastPos = 0;

    public Turret(LinearOpMode lom) {
        s1 = lom.hardwareMap.get(CRServo.class, RobotConstants.TurretServo1);
        s2 = lom.hardwareMap.get(CRServo.class, RobotConstants.TurretServo2);

        s1En = lom.hardwareMap.get(AnalogInput.class, RobotConstants.TurretServoEncoder1);
        s2En = lom.hardwareMap.get(AnalogInput.class, RobotConstants.TurretServoEncoder2);

        s1.setDirection(DcMotorSimple.Direction.FORWARD);
        g = lom.gamepad1;
        t = lom.telemetry;
        lastPos = s1En.getVoltage();
    }

    public void TeleOp() {
        s1.setPower(g.right_stick_x);
        s2.setPower(g.right_stick_x);
        t.addData("servo 1 output:", s1En.getVoltage());
        t.addData("servo 2 output:", s2En.getVoltage());
        t.addData("servo 2 psewvdo angle output:", s1En.getVoltage() / 3.3 * 355.0);
        t.addData("servo 2 psevdo angle output:", s2En.getVoltage() / 3.3 * 355.0);
    }

    public void AutoAiming(double bearing) {
        double currentServoAngle = GetCurrentPosition();
        double targetServoAngle = Math.min(Math.max(bearing * cameraMultiply + currentServoAngle, minAngle), maxAngle);
        double power = PID(currentServoAngle, targetServoAngle);
        s1.setPower(power);
        s2.setPower(power);
    }

    private double PID(double target, double current) {
        double current_time = pidTimer.milliseconds();
        double error = target - current;

        // 2. Calculate time change; skip loop if delta is too small to be reliable.
        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) {
            return 0;
        }

        // 3. Calculate P, I, D components
        double pComponent = Shooter.Configuration.PID_COEFFICIENTS.p * error;
        double derivative = (error - lastError) / deltaTime;
        double dComponent = Shooter.Configuration.PID_COEFFICIENTS.d * derivative;

        // 4. Add Feed-Forward (F) component

        // 5. Sum all components to get the final output power.
        pidOutput = pComponent + dComponent;

        // 8. Save state for the next loop iteration.
        lastError = error;
        lastTime = current_time;
        return pidOutput;
    }

    private double GetCurrentPosition() {
        double currentPos = s1En.getVoltage();
        if(Math.abs(currentPos-lastPos) > 2.5)
            if(Math.abs(lastPos-leftPose)<0.1)
                countOfFullTurn--;
            else
                countOfFullTurn++;
        lastPos = currentPos;
        return currentPos / 3.3 * servoRange + countOfFullTurn * servoRange;
    }
}
