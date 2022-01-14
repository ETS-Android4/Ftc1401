package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    int leftbumper;
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    LinearOpMode L;
    DcMotor Lm, Rm, Duck, Claw, Cran;
    Gamepad gamepad1,gamepad2;
    Servo brush, backet;

    Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode L, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.L = L;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }

    public void init() {
        Lm = hardwareMap.get(DcMotor.class, "L");
        Rm = hardwareMap.get(DcMotor.class, "R");
        Duck = hardwareMap.get(DcMotor.class, "D");
//        brush = hardwareMap.get(Servo.class, "brush");
//        Claw = hardwareMap.get(DcMotor.class, "Claw");
//        backet = hardwareMap.get(Servo.class, "backet");

        Lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lm.setDirection(DcMotorSimple.Direction.REVERSE);
        Duck.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            delay(30);
            Telemetry.Item item = telemetry.addData("Wait", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("Ok", "Calibrated");
        telemetry.update();
    }
    public final void delay(long milliseconds) {
        try {
            sleep(milliseconds);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void drive() {
        Rm.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        Lm.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        Duck.setPower(gamepad1.right_trigger);
        Duck.setPower(-gamepad1.left_trigger);




//        Cran.setPower(gamepad1.left_stick_y);
//        Cran.setPower(gamepad1.left_stick_x);

//        brush.setPosition(gamepad1.right_trigger);
//        delay(10);
//        brush.setPosition(gamepad1.right_trigger);
//
//        backet.setPosition(gamepad1.right_stick_x);
//        delay(10);
//        backet.setPosition(gamepad1.right_stick_y);
    }


    public void goForward(int kmm) {
        Rm.setDirection(DcMotorSimple.Direction.FORWARD);
        Lm.setDirection(DcMotorSimple.Direction.REVERSE);

        Lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int r = 48;
        double tick = 1 / (2 * Math.PI * r ) * 537.7 * kmm;
        while (Lm.getCurrentPosition() <= tick ) {

            Lm.setPower(1);
            Rm.setPower(1);

            telemetry.addData("ticks", Lm.getCurrentPosition());
            telemetry.addData("ticksVar", tick);
            telemetry.update();
        }

        Rm.setDirection(DcMotorSimple.Direction.REVERSE);
        Lm.setDirection(DcMotorSimple.Direction.FORWARD);
        Lm.setPower(1);
        Rm.setPower(1);
        delay(10);

        telemetry.addData("OK", tick);
        telemetry.update();
        Lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void goBack(int kmm) {
        Rm.setDirection(DcMotorSimple.Direction.REVERSE);
        Lm.setDirection(DcMotorSimple.Direction.FORWARD);

        Lm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int r = 48;
        double tick = 1 / (2 * Math.PI * r ) * 537.7 * kmm;
        while (Lm.getCurrentPosition() <= tick ) {

            Lm.setPower(1);
            Rm.setPower(1);

            telemetry.addData("ticks", Lm.getCurrentPosition());
            telemetry.addData("ticksVar", tick);
            telemetry.update();
        }

        Rm.setDirection(DcMotorSimple.Direction.FORWARD);
        Lm.setDirection(DcMotorSimple.Direction.REVERSE);
        Lm.setPower(1);
        Rm.setPower(1);
        delay(10);

        telemetry.addData("OK", tick);
        telemetry.update();
        Lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnRight(int degrees){
        while (degrees != getAngle()){
            Rm.setDirection(DcMotorSimple.Direction.FORWARD);
            Lm.setDirection(DcMotorSimple.Direction.FORWARD);
            Lm.setPower(1);
            Rm.setPower(1);
        }
        telemetry.addData("angle", getAngle());
        telemetry.update();
        Lm.setPower(0);
        Rm.setPower(0);
    }

    public void turnLeft(int degrees){
        while (degrees != getAngle()){
            Rm.setDirection(DcMotorSimple.Direction.REVERSE);
            Lm.setDirection(DcMotorSimple.Direction.REVERSE);
            Lm.setPower(1);
            Rm.setPower(1);
        }
        telemetry.addData("angle", getAngle());
        telemetry.update();
        Lm.setPower(0);
        Rm.setPower(0);
    }
}
