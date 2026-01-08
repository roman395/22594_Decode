package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class AutonomRedBig extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private ElapsedTime delayTimer = new ElapsedTime();
    private boolean isWaiting = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(129, 110, Math.toRadians(-90)));
        // ОБЯЗАТЕЛЬНО: сброс одометрии перед началом


        paths = new Paths(follower);
        pathState = 0;
        isWaiting = false;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();

        // ВСЕГДА обновляем state machine, но внутри учитываем isWaiting
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Waiting", isWaiting);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.debug("Delay Timer", delayTimer.seconds());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0: // НАЧАЛЬНОЕ СОСТОЯНИЕ - только один раз!
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootPreload);
                    return 1; // Переходим в состояние ожидания завершения
                }
                break;

            case 1: // Ждем завершения ShootPreload
                if (!follower.isBusy()) {
                    startDelay();
                    return 2; // Переходим в состояние задержки
                }
                break;

            case 2: // СОСТОЯНИЕ ЗАДЕРЖКИ после ShootPreload
                if (isWaiting) {
                    // Ждем пока не пройдет 2 секунды
                    if (delayTimer.seconds() >= 2.0) {
                        isWaiting = false; // Завершаем задержку
                        delayTimer.reset();
                        return 3; // Переходим к следующему пути
                    }
                } else {
                    // Если isWaiting = false, значит задержка еще не начата
                    startDelay();
                }
                break;

            case 3: // Начинаем TakeSpike1
                if (!follower.isBusy()) {
                    follower.followPath(paths.TakeSpike1);
                    return 4;
                }
                break;

            case 4: // Ждем завершения TakeSpike1
                if (!follower.isBusy()) {
                    startDelay();
                    return 5;
                }
                break;

            case 5: // Задержка после TakeSpike1
                if (isWaiting) {
                    if (delayTimer.seconds() >= 2.0) {
                        isWaiting = false;
                        delayTimer.reset();
                        return 6;
                    }
                } else {
                    startDelay();
                }
                break;

            case 6: // Начинаем Shoot1
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot1);
                    return 7;
                }
                break;

            case 7: // Ждем завершения Shoot1
                if (!follower.isBusy()) {
                    startDelay();
                    return 8;
                }
                break;

            case 8: // Задержка после Shoot1
                if (isWaiting) {
                    if (delayTimer.seconds() >= 2.0) {
                        isWaiting = false;
                        delayTimer.reset();
                        return 9;
                    }
                } else {
                    startDelay();
                }
                break;

            case 9: // Начинаем TakeSpike2
                if (!follower.isBusy()) {
                    follower.followPath(paths.TakeSpike2);
                    return 10;
                }
                break;

            case 10: // Ждем завершения TakeSpike2
                if (!follower.isBusy()) {
                    startDelay();
                    return 11;
                }
                break;

            case 11: // Задержка после TakeSpike2
                if (isWaiting) {
                    if (delayTimer.seconds() >= 2.0) {
                        isWaiting = false;
                        delayTimer.reset();
                        return 12;
                    }
                } else {
                    startDelay();
                }
                break;

            case 12: // Начинаем Shoot2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2);
                    return 13;
                }
                break;

            case 13: // Ждем завершения Shoot2
                if (!follower.isBusy()) {
                    startDelay();
                    return 14;
                }
                break;

            case 14: // Задержка после Shoot2
                if (isWaiting) {
                    if (delayTimer.seconds() >= 2.0) {
                        isWaiting = false;
                        delayTimer.reset();
                        return 15;
                    }
                } else {
                    startDelay();
                }
                break;

            case 15: // Начинаем Leave
                if (!follower.isBusy()) {
                    follower.followPath(paths.Leave);
                    return 16;
                }
                break;

            case 16: // ФИНАЛЬНОЕ СОСТОЯНИЕ
                // Автоном завершен
                if (!follower.isBusy()) {
                    // Можно остановить моторы для уверенности
                    // follower.stop();
                    panelsTelemetry.debug("Status", "Autonomous Complete!");
                }
                break;

            default:
                pathState = 0; // Сброс на случай ошибки
        }
        return pathState;
    }

    private void startDelay() {
        isWaiting = true;
        delayTimer.reset();
        panelsTelemetry.debug("Delay", "Started (2s)");
    }

    // Внутренний класс Paths (без изменений)

    public static class Paths {
        public PathChain ShootPreload;
        public PathChain TakeSpike1;
        public PathChain Shoot1;
        public PathChain TakeSpike2;
        public PathChain Shoot2;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.000, 110.000),

                                    new Pose(107.000, 120.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-160))

                    .build();

            TakeSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(107.000, 120.000),
                                    new Pose(122.000, 123.000),
                                    new Pose(120.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(-90))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.000, 90.000),

                                    new Pose(108.000, 97.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-130))

                    .build();

            TakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(108.000, 97.000),
                                    new Pose(118.736, 95.615),
                                    new Pose(120.000, 65.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(-90))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.000, 65.000),

                                    new Pose(99.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-130))

                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.000, 88.000),

                                    new Pose(99.000, 80.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-130))

                    .build();
        }
    }

}