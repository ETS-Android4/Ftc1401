package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class A extends LinearOpMode {
    DcMotor MotorA;
    Servo ServoA;
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
       MotorA = hardwareMap.get(DcMotor.class, "MotorA");
       ServoA = hardwareMap.get(Servo.class, "ServoA");
       MotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       initIMU();
       waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("IMU", getAngle());
            telemetry.update();
        }

       ServoA.setPosition(0);
       sleep(1000);
       ServoA.setPosition(1);
    }
    public void Move1 (int testMotor) {
        int op = MotorA.getCurrentPosition();
        while (MotorA.getCurrentPosition() - op < testMotor) {
            MotorA.setPower(1);
            telemetry.addData("Sheeeesh", MotorA.getCurrentPosition());
            telemetry.update();
        }
        MotorA.setPower(0);
    }
     public void initIMU() {
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
             sleep(20);
             telemetry.addData("Wait", "Calibrating");
             telemetry.update();
         }
         telemetry.addData("Ok", "Calibrated");
         telemetry.update();
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void telemetry() {




    }
}
