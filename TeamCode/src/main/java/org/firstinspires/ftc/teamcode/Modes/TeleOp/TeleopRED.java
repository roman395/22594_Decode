package org.firstinspires.ftc.teamcode.Modes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp RED", group = "Main")
public class TeleopRED extends TeleOpBase {
    @Override
    protected int getAprilTagId() {
        return 24;
    }

    @Override
    protected String getAllianceName() {
        return "RED";
    }
}