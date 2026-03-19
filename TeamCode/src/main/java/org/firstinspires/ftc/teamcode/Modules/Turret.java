package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

@Configurable
public class Turret {
    private CRServo s1, s2;
    private AnalogInput s1En, s2En;
    private Gamepad g;
    public static double maxAngle = 145, maxIntegral = 0.3, minAngle = -123, cameraMultiply = 1, servoRange = 370, offset = 0;
    private boolean isCloseToBreake = false;
    public static double breakPose = 3.2;
    public static double centerPose = 2.736;
    public static double inputMultiply = 0.5;
    private boolean iCanSee = false;

    // Константы для детекта переходов
    private static final double MAX_VOLTAGE = 3.3;
    private static final double LOW_VOLTAGE_THRESHOLD = 0.2;      // В вольтах
    private static final double HIGH_VOLTAGE_THRESHOLD = 3.1;    // В вольтах

    // Переводной коэффициент: (вольтаж) -> (угол с учётом редукции)
    // При полном обороте датчика (3.3В) мы получаем servoRange градусов на выходе
    private static final double VOLTAGE_TO_ANGLE = servoRange / MAX_VOLTAGE;

    private Telemetry t;
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime seenTimer = new ElapsedTime();
    private double lastTime, countOfFullTurn = 0, integral = 0, pidOutput, lastError = 0;
    private double lastVoltage = 0;  // Храним предыдущее напряжение для детекта переходов
    private double startPose = 0;
    private double targetPose = 0;

    public Turret(LinearOpMode lom) {
        s1 = lom.hardwareMap.get(CRServo.class, RobotConstants.TurretServo1);
        s2 = lom.hardwareMap.get(CRServo.class, RobotConstants.TurretServo2);

        s1En = lom.hardwareMap.get(AnalogInput.class, RobotConstants.TurretServoEncoder1);
        s2En = lom.hardwareMap.get(AnalogInput.class, RobotConstants.TurretServoEncoder2);

        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        s1.setPower(1);
        s2.setPower(1);
        s1.setPower(0);
        s2.setPower(0);
        g = lom.gamepad1;
        t = lom.telemetry;

        // Инициализация
        lastVoltage = s1En.getVoltage();
        startPose = lastVoltage;
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
        t.addData("servo 1 voltage:", s1En.getVoltage());
        t.addData("servo 2 voltage:", s2En.getVoltage());
        t.addData("turret angle:", currentPos);
        t.addData("full rotations:", countOfFullTurn);
        t.addData("center pose", centerPose);
    }

    public void AutoAimingOnTarget(double bearing) {
        double currentServoAngle = GetCurrentPosition();
        double targetServoAngle = Math.clamp(bearing * cameraMultiply, minAngle, maxAngle);
        double power = PIDOnTarget(targetServoAngle, currentServoAngle);
        s1.setPower(power);
        s2.setPower(power);
    }

    public void AutoAimingOnError(double error) {
        // 1. Сначала проверяем, валидная ли ошибка
        if (error == -999999 || Math.abs(error) > 180) { // Ограничь максимальный поворот за раз
            // Останавливаем моторы, если нет цели
            s1.setPower(0);
            s2.setPower(0);
            return;
        }

        double currentServoAngle = GetCurrentPosition();

        // 2. Проверяем, не выйдем ли за пределы после поворота
        double targetAngle = currentServoAngle + error;

        // Нормализуем targetAngle в диапазон [-180..180] относительно центра
        // чтобы корректно сравнивать с minAngle и maxAngle
        // 3. Если целевой угол в пределах допустимого
        if (targetAngle >= minAngle && targetAngle <= maxAngle) {
            double power = PIDOnError(error, currentServoAngle);
            s1.setPower(power);
            s2.setPower(power);
        } else {
            // Если выходим за пределы - не двигаемся
            s1.setPower(0);
            s2.setPower(0);
        }
    }

    private double PIDOnTarget(double target, double current) {
        double current_time = pidTimer.milliseconds();
        double error = target - current + offset;

        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) {
            return 0;
        }

        double pComponent = RobotConstants.TurretPid.p * error;
        integral += error * deltaTime;
        integral = Math.clamp(integral, -maxIntegral, maxIntegral);
        double iComponent = integral * RobotConstants.TurretPid.i;
        double derivative = (error - lastError) / deltaTime;
        double dComponent = RobotConstants.TurretPid.d * derivative;

        pidOutput = pComponent + dComponent + iComponent;

        lastError = error;
        lastTime = current_time;
        return pidOutput;
    }

    private double PIDOnError(double error, double current) {
        error += offset;
        double current_time = pidTimer.milliseconds();
        double deltaTime = (current_time - lastTime);
        if (deltaTime < 1) {
            return 0;
        }

        double pComponent = RobotConstants.TurretPid.p * error;
        integral += error * deltaTime;
        integral = Math.clamp(integral, -maxIntegral, maxIntegral);
        double iComponent = integral * RobotConstants.TurretPid.i;
        double derivative = (error - lastError) / deltaTime;
        double dComponent = RobotConstants.TurretPid.d * derivative;
        pidOutput = pComponent + dComponent + iComponent;

        lastError = error;
        lastTime = current_time;
        return pidOutput;
    }

    /**
     * Получить текущий угол турели с учётом всех оборотов
     * Возвращает угол в градусах (может быть больше 370 или отрицательным)
     */
    public double GetCurrentPosition() {
        double currentVoltage = s1En.getVoltage();
        currentVoltage = Math.min(MAX_VOLTAGE, Math.max(0, currentVoltage));

        // Вычисляем изменение
        double voltageChange = currentVoltage - lastVoltage;

        // Если изменение больше половины диапазона - значит перескочили через границу
        if (voltageChange > MAX_VOLTAGE / 2) {
            // Было мало, стало много? Нет, наоборот: если изменение положительное и большое,
            // значит мы перешли с ~0 на ~3.3 (движение назад)
            countOfFullTurn--; // Движение назад
        } else if (voltageChange < -MAX_VOLTAGE / 2) {
            // Отрицательное большое изменение = перешли с 3.3 на 0 (движение вперед)
            countOfFullTurn++; // Движение вперед
        }

        // Проверка на близость к "точке разрыва" для замедления
        isCloseToBreake = Math.abs(currentVoltage - breakPose) < 0.5;

        lastVoltage = currentVoltage;

        return (currentVoltage - centerPose) * VOLTAGE_TO_ANGLE + countOfFullTurn * servoRange;
    }

    /**
     * Сброс нулевой позиции (калибровка)
     */
    public void resetStartPose() {
        centerPose = s1En.getVoltage();
        countOfFullTurn = 0;
        lastVoltage = centerPose; // Важно! Обновляем lastVoltage при сбросе
    }

    public double getS1Pos() {
        return s1En.getVoltage();
    }

    public double getRawAngle() {
        double voltage = s1En.getVoltage();
        voltage = Math.min(MAX_VOLTAGE, Math.max(0, voltage));
        return voltage * VOLTAGE_TO_ANGLE;
    }

    public void advancedTelemetry(Telemetry telemetry) {
        telemetry.addData("Servo pos", s1En.getVoltage());
        telemetry.addData("Current Angle", GetCurrentPosition());
        telemetry.addData("Current center pose", centerPose);
        telemetry.addData("Current center pose", centerPose);
    }

    public void updateTurret() {
        s1.setPower(PIDOnTarget(targetPose, GetCurrentPosition()));
        s2.setPower(PIDOnTarget(targetPose, GetCurrentPosition()));
    }

    public void setTargetPose(double targetPose) {
        this.targetPose = targetPose;
    }

    public void goToStart() {
        targetPose = startPose;
    }
}