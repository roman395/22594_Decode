package org.firstinspires.ftc.teamcode.Deprecated;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class RobotConstants {
    public static final String MecanumFL = "FLM",
            MecanumFR = "FRM",
            MecanumRL = "RLM",
            MecanumRR = "RRM",
            StrafeEncoder = "RRM",
            ForwardEncoder = "FRM",
            Camera = "camera",
            IntakeMotor = "intake",
            FeederMotor = "feeder",
            TurretServo1 = "tServo1",
            TurretServo2 = "tServo2",
            TurretServoEncoder1 = "tServoEn1",
            TurretServoEncoder2 = "tServoEn2",
            RevDistanceSensor = "sensor",
            ShootServo = "shootS",
            ShootLeft = "shootL",
            ShootRight = "shootR";
    public static PIDFCoefficients ShooterPid = new PIDFCoefficients(0.003, 0, 0, 0.00036);
    public static PIDFCoefficients TurretPid = new PIDFCoefficients(0.005, 0, 0.0005, 0.08);

    public static long CameraExposure = 0;
    public static double DistanceToFarCriticalError = 1500;
    public static double CloseCriticalError = 50;
    public static double FarCriticalError = 110;

}
