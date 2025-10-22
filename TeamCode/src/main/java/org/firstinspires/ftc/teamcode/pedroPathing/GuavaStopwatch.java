package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.util.concurrent.TimeUnit.MILLISECONDS;

import com.google.firebase.crashlytics.buildtools.reloc.com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "GuavaTest", group = "Over-caffeinated")
public class GuavaStopwatch extends OpMode {
    //This is an example. When we get TeleOp on here, you can add this.

    private com.google.firebase.crashlytics.buildtools.reloc.com.google.common.base.Stopwatch stopwach;
    private Stopwatch stopwatch = Stopwatch.createUnstarted();
    long millis = stopwach.elapsed(MILLISECONDS);
    long timeLeftInSeconds = 150 - (millis/1000);
    long timeLeftInMinutes = (timeLeftInSeconds-timeLeftInSeconds%60)/60;

    @Override
    public void init(){

        System.out.println(timeLeftInMinutes+":"+timeLeftInSeconds%60);
    }

    @Override
    public void loop() {
        System.out.println(timeLeftInMinutes+":"+timeLeftInSeconds%60);
    }
}
