package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

    public class cont extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot R = new Robot(hardwareMap, telemetry, this, gamepad1, gamepad2);
        R.init();
        waitForStart();
        while (!isStopRequested()) {
            R.drive();
        }
    }

}
