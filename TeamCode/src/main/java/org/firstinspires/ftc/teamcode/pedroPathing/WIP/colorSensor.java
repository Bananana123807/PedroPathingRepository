package org.firstinspires.ftc.teamcode.pedroPathing.WIP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Color Sensor")
public class colorSensor extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Make sure this name matches your configuration exactly (no spaces, case-sensitive)
        colorSensor = hardwareMap.get(ColorSensor.class, "color Sensor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        String ballColor = "ballColorNotFound";

        waitForStart();

        while (opModeIsActive()) {
            // Read RGB values from the sensor
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Display raw RGB values
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);

            // Determine and display detected color
            if (isGreen(red, green, blue)) {
                ballColor = "Green";
                telemetry.addData("Detected Color:", ballColor);
            } else if (isPurple(red, green, blue)) {
                ballColor = "Purple";
                telemetry.addData("Detected Color:", ballColor);
            } else {
                telemetry.addLine("Detected Color: Unknown");
            }


            telemetry.update();
        }
    }

    private boolean isGreen(int red, int green, int blue) {
        return green > red + 20 && green > blue + 20 && green > 100;
    }

    private boolean isPurple(int red, int green, int blue) {
        return red>80 && blue>80 && green < 60;
    }
    private boolean isBlack(int red, int green, int blue){
        return red<20 && blue>80 && green>10;
    }
}



