package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Camera.AprilTagsDetection;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@Configurable
public class Shooter {
    public static double approxVelCof = 0.2303, approxWallCof = 0.00058, approxVelOf = 1535.3528, approxWallOf = 0.08927;
    DcMotorEx leftM, rightM, forwardEncoder, strafeEncoder;
    public static double velocity = 2000;
    public static double pos = 0;
    public AprilTagsDetection detect;
    int current_pos = 0;
    Servo rightS, leftS;
    Gamepad g;
    Telemetry t;
    public static PIDFCoefficients pid = new PIDFCoefficients(0.001, 0.0001, 0.0009, 0);
    // ВАЖНО: Добавлен коэффициент Feed-Forward (F)
    public static double feedForward = 0.78; // Настраиваемое значение F

    ElapsedTime timer = new ElapsedTime();
    private double current_velocity = 1800;
    private boolean isShooting = false, isFirstIntake = false, isRumble;
    ElapsedTime pidTimer = new ElapsedTime();
    ExposureControl control;
    public static boolean useCamera = true, usePhysic = false;

    // Добавлены константы для PID
    private static final double MAX_POWER = 0.99; // Максимальная мощность для защиты моторов
    private double integralSum = 0.0; // Переменная для накопления интегральной компоненты
    private double lastError = 0.0;   // Последняя ошибка для расчета производной
    private double lastTime = 0.0;    // Время последнего вызова PID

    // PID переменные (теперь используются более логично)
    private double current_time = 0, current_error = 0, output = 0;

    public Shooter(LinearOpMode lom, Telemetry tel) {
        g = lom.gamepad1;
        t = tel;

        leftM = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootLeft);
        rightM = lom.hardwareMap.get(DcMotorEx.class, RobotConstants.ShootRight);
        rightS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoR);
        leftS = lom.hardwareMap.get(Servo.class, RobotConstants.ShootServoL);
        leftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
        forwardEncoder = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.ForwardEncoder);
        strafeEncoder = lom.hardwareMap.get(DcMotorImplEx.class, RobotConstants.StrafeEncoder);

        leftS.setDirection(Servo.Direction.REVERSE);
        detect = new AprilTagsDetection(lom, t);
        getVisionPortal().resumeStreaming();
        getVisionPortal().resumeLiveView();
        PanelsCameraStream.INSTANCE.startStream(getVisionPortal(), 30);
        control = getVisionPortal().getCameraControl(ExposureControl.class);
        control.setMode(ExposureControl.Mode.Manual);
        control.setExposure(3, TimeUnit.MILLISECONDS);
    }

    private void Telemetry() {
        if (useCamera)
            t.addData("distance", detect.GetDistance(24));
        t.addData("right velocity", rightM.getVelocity());
        t.addData("left velocity", leftM.getVelocity());
        t.addData("current velocity", current_velocity);
        t.addData("error", current_error);
        t.addData("output power", output);
        t.addData("left motor power", leftM.getPower());
        t.addData("right motor power", rightM.getPower());
        t.addData("wall servo pos", pos);
        t.addData("feedForward", feedForward);
        t.addData("integralSum", integralSum);
    }

    public double VelocityTests(double velocity, double pos, int id, boolean isCameraUse) {
        detect.Update();
        t.addData("distance", detect.GetDistance(id));
        if (isCameraUse) {
            PID(rightM, leftM, DistToVelocity(detect.GetDistance(id)));
            leftS.setPosition(DistToWallPose(detect.GetDistance(id)));
        } else
            PID(rightM, leftM, velocity);
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        return current_error;
    }

    public double TeleOp(Intake intake, int id) {
        detect.Update();

        // Получаем целевые значения
        if (detect.GetDistance(id) != -1 && useCamera) {
            if (!isShooting || g.left_stick_x != 0 || g.left_stick_y != 0 || (g.right_trigger - g.left_trigger) != 0) {
                current_velocity = DistToVelocity(detect.GetDistance(id));
                pos = DistToWallPose(detect.GetDistance(id));
            }
        } else if (!useCamera && !usePhysic) {
            current_velocity = velocity;
        }

        // Управление серво
        leftS.setPosition(pos);
        rightS.setPosition(pos);

        // Управление шутером
        if (isShooting) {
            PID(rightM, leftM, current_velocity);
        }

        // Логика кнопок для стрельбы
        if (g.rightBumperWasReleased()) timer.reset();
        if (g.rightBumperWasPressed() && !isFirstIntake) isFirstIntake = true;

        if (g.circle) {
            isShooting = true;
        } else if (g.square) {
            leftM.setPower(0);  // Исправлено: было -0
            rightM.setPower(0); // Исправлено: было -0
            isShooting = false;
            integralSum = 0;    // Сбрасываем интеграл при остановке
        }

        // Обратная подача шариков (reverse feed)
        if (!isShooting && (g.right_bumper || timer.milliseconds() < 1500) && isFirstIntake) {
            leftM.setPower(-0.6);
            rightM.setPower(-0.6);
            integralSum = 0; // Сбрасываем интеграл при реверсе
        }

        // Управление интейком (подачей шариков)
        boolean shouldFeed = false;
        if (g.right_bumper && !isShooting) {
            shouldFeed = true;
        } else if (isShooting && g.right_bumper && Math.abs(current_error) < 110) {
            // Критически важно: подаем ТОЛЬКО когда скорость стабилизирована
            shouldFeed = true;
        }

        if (shouldFeed) {
            intake.ShooterEnable(1);
        } else if (g.left_bumper) {
            intake.ShooterEnable(-0.6);
        } else {
            intake.ShooterEnable(0);
        }

        // Остановка моторов когда не нужно
        if (!g.right_bumper && !isShooting && !g.triangle) {
            leftM.setPower(0);
            rightM.setPower(0);
            integralSum = 0;
        }

        // Режим медленного вращения для отладки
        if (g.triangle && !isShooting) {
            leftM.setPower(-0.5);
            rightM.setPower(-0.5);
            integralSum = 0;
        }

        // Настройка параметров с джойстика
        if (!useCamera) {
            g.setLedColor(255, 0, 0, 1000);
            if (g.dpadLeftWasPressed())
                current_velocity -= 50;
            else if (g.dpadRightWasPressed())
                current_velocity += 50;
            if (g.dpadUpWasPressed())
                pos += 0.05;
            else if (g.dpadDownWasPressed())
                pos -= 0.05;
        } else {
            g.setLedColor(0, 255, 0, 1000);

        }

        Telemetry();
        return detect.GetBearing(id);
    }

    public void ResetTimer() {
        timer.reset();
    }

    public void Update() {
        detect.Update();
    }

    public boolean Autonom(double time, int id) {
        if (detect.GetDistance(id) != -1) {
            current_velocity = DistToVelocity(detect.GetDistance(id));
            pos = DistToWallPose(detect.GetDistance(id));
        }
        leftS.setPosition(pos);
        rightS.setPosition(pos);
        if (time > timer.milliseconds()) {
            PID(rightM, leftM, current_velocity);
            return false;
        } else {
            rightM.setPower(0);
            leftM.setPower(0);
            integralSum = 0;
            return true;
        }
    }

    // ИСПРАВЛЕННЫЙ PID-РЕГУЛЯТОР
    private void PID(DcMotorEx motorR, DcMotorEx motorL, double targetVelocity) {
        // 1. Получаем текущие данные
        current_time = pidTimer.milliseconds();
        double currentVelocity = (motorL.getVelocity() + motorR.getVelocity()) / 2.0;
        current_error = targetVelocity - currentVelocity;

        // 2. Вычисляем изменение времени (защита от деления на 0)
        double deltaTime = (current_time - lastTime);
        if (deltaTime == 0) deltaTime = 1;

        // 3. Рассчитываем P, I, D компоненты
        // P-компонента: пропорциональна текущей ошибке
        double pComponent = pid.p * current_error;

        // I-компонента: накапливаем ошибку со временем (интеграл)
        integralSum += (current_error * deltaTime);
        // Антивиндкап: ограничиваем интеграл
        if (integralSum > 0.5) integralSum = 0.5;
        else if (integralSum < -0.5) integralSum = -0.5;
        double iComponent = pid.i * integralSum;

        // D-компонента: скорость изменения ошибки (производная)
        double derivative = (current_error - lastError) / deltaTime;
        double dComponent = pid.d * derivative;

        // 4. КРИТИЧЕСКИ ВАЖНО: Добавляем Feed-Forward (F)
        // Это базовая мощность для быстрого достижения целевой скорости
        double feedForwardPower = 0;
        if (targetVelocity != 0 && feedForward > 0) {
            feedForwardPower = feedForward;
        }

        // 5. Суммируем все компоненты (F + PID)
        output = feedForwardPower + pComponent + iComponent + dComponent;

        // 6. Ограничиваем выходную мощность для защиты моторов
        if (output > MAX_POWER) output = MAX_POWER;
        if (output < 0) output = 0; // Для шутера обычно не нужна отрицательная мощность

        // 7. Применяем мощность к моторам
        if (targetVelocity != 0) {
            motorR.setPower(output);
            motorL.setPower(output);
        } else {
            motorR.setPower(0);
            motorL.setPower(0);
            integralSum = 0; // Сбрасываем интеграл при остановке
        }

        // 8. Сохраняем значения для следующего цикла
        lastError = current_error;
        lastTime = current_time;
    }

    private double DistToVelocity(double dist) {
        return -0.00000832850904453064 * dist * dist + 0.16237677658523352875 * dist + 1729.16030889091780409217;
    }

    private double DistToWallPose(double dist) {
        return Math.min(0.00035929030903905293 * dist -0.24707690788092798173, 0.95);
    }

    public VisionPortal getVisionPortal() {
        return detect.getPortal();
    }

    private double DistToLaunchAngle(double elevation) {
        return Math.atan(Math.tan(Math.toRadians(elevation)));
    }

    private double LaunchAngleToServo(double angle) {
        return Math.toDegrees(angle) / 180 * 0.95;
    }

    private double DistToVelocityPhysic(double dist, double launchAngle) {
        return Math.sqrt(9.81 * dist / Math.sin(2 * launchAngle));
    }

    private double VelocityToTPS(double velocity) {
        return (28 * 28 / 22.0 * velocity) / (Math.PI * 0.072);
    }
}