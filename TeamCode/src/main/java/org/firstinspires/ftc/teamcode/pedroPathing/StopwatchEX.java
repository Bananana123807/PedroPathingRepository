package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class StopwatchEX extends OpMode {
    private final ElapsedTime bananas = new ElapsedTime();
    private double timeLeft = 150 - bananas.seconds();

    @Override
    public void init() {
        bananas.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Time in seconds", bananas.seconds());

    }
}
