package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "autoRed")

public class auto1 extends LinearOpMode {
    Robot R;
    DcMotor Lm, Rm; //, ClawLift, Duck;
    //Servo ClawServo;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        Lm = hardwareMap.get(DcMotor.class, "Left");
        Rm = hardwareMap.get(DcMotor.class, "Right");
        //ClawLift = hardwareMap.get(DcMotor.class, "ClawLift");
       // Duck = hardwareMap.get(DcMotor.class, "Duck");
        //ClawServo = hardwareMap.get(Servo.class, "ClawServo");

        Lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Rm.setDirection(DcMotorSimple.Direction.FORWARD);
        Lm.setDirection(DcMotorSimple.Direction.REVERSE);

       // ClawLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        R.goForward(200);
        R.turnRight(90);
        R.goBack(200);
        R.turnLeft(90);

    }

}

