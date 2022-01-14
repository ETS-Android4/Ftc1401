package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="viking", group="Linear Opmode")
public class viking extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    Servo servo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        servo = hardwareMap.get(Servo.class, "servo_one");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower_p;
            double rightPower_p;
            double leftPower_r;
            double rightPower_r;

            leftPower_p  = -gamepad1.left_stick_y ;
            leftPower_r  = -gamepad1.left_stick_x ;
            rightPower_p = gamepad1.left_stick_y ;
            rightPower_r = -gamepad1.left_stick_x ;

            leftDrive.setPower(leftPower_p);
            rightDrive.setPower(rightPower_p);
            leftDrive.setPower(leftPower_r);
            rightDrive.setPower(rightPower_r);


            if (gamepad1.x) {
                servo.setPosition(0.5);
            }
            if (gamepad1.b) {
                servo.setPosition(-0.5);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower_p, rightPower_p);
            telemetry.update();
        }
    }
}
