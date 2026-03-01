package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp
@Config
public class NewMainTeleOp extends OpMode {
    MecanumDrive drive;
//    DcMotorImplEx intakeMotor;
    DcMotorImplEx outtakeMotor1;
    DcMotorImplEx outtakeMotor2;
    DcMotorImplEx intakeMotor;
    MotorEx outtakeMotorEx;
    CRServoImplEx servo1;
    Servo servo2;
    ControlSystem controller;
    boolean farOrClose;
    double motorPower;
    boolean servoOn;
    boolean servoDirection;
    double output;
    boolean adjustable;
    public static int target;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.01, 0.01, 0.01);
    public static double kS, kV, kA;

    @Override
    public void init() {
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        outtakeMotor1 = hardwareMap.get(DcMotorImplEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorImplEx.class, "outtakeMotor2");
//        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        farOrClose = true; // true is far, false is close
//        servo1 = hardwareMap.get(CRServoImplEx.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servoOn = false;
        adjustable = false;
        output = 0;


        servoDirection = false;
        motorPower = .5;

        controller = ControlSystem.builder()
                .velPid(pidCoefficients)
                .basicFF(kV)
                .build();

    }

    @Override
    public void loop() {

//        if (gamepad1.dpad_left) {
//            adjustable = !adjustable;
//        }
//        if (outtakeMotor.getVelocity() >= 1600 && farOrClose) {
//            gamepad1.rumble(200);
//        } else if (outtakeMotor.getVelocity() >= 1200) {
//            gamepad1.rumble( 200);
//        }
//        if (adjustable) {
//            if (gamepad1.right_trigger > 0.1) {
//                outtakeMotor.setPower(motorPower);
//            } else {
//                outtakeMotor.setPower(0);
//            }
//            if (gamepad1.aWasReleased()) {
//                if (motorPower <= .99) {
//                    motorPower += .01;
//                }
//            }
//
//            if (gamepad1.bWasReleased()) {
//                if (outtakeMotor.getPower() >= .1) {
//                    motorPower -= .01;
//                }
//            }
//        } else {
            if (gamepad1.right_trigger > 0.1) {
                if (farOrClose) {
                    outtakeMotor1.setPower(1);
                    outtakeMotor2.setPower(1);
                } else {
                    outtakeMotor1.setPower(.67);
                    outtakeMotor2.setPower(.67);
                }
            } else {
                outtakeMotor1.setPower(0);
                outtakeMotor2.setPower(0);
            }
//        }
//
//        if (gamepad1.options) {
//            adjustable = !adjustable;
//        }

        if (gamepad1.dpad_up) {
            farOrClose = true;
        } else if (gamepad1.dpad_down) {
            farOrClose = false;
        }

        if (gamepad1.a) {
            servo2.setPosition(.4);
        }

        if (gamepad1.x) {
            servo2.setPosition(0);
        }


//        if (gamepad1.xWasPressed()) {
//                servo1.setPower(.75);
//        }
//        else if (gamepad1.xWasReleased())
//        {
//            servo1.setPower(0);
//        }


//        if (gamepad1.left_trigger > 0.1) {
//            intakeMotor.setPower(.85);
//        } else if (gamepad1.b){
//            intakeMotor.setPower(-.67);
//        } else {
//            intakeMotor.setPower(0);
//        }
//
//        if(gamepad1.left_bumper)
//        {
//            intakeMotor.setPower(.8);
//            servo1.setPower(-.3);
//
//        } else if(gamepad1.left_trigger<.1 && !gamepad1.cross && !gamepad1.square)
//        {
//            intakeMotor.setPower(0.0);
//            servo1.setPower(0.0);
//        }



//        double y = -gamepad1.left_stick_y ; // negate to make outtake front
//        double x = -gamepad1.left_stick_x; // negate to make outtake front
//        double rx = -gamepad1.right_stick_x;
//
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, x), rx));
//        drive.updatePoseEstimate();
//
//        telemetry.addLine("LOCALIZATION");
//        telemetry.addData("X Position", drive.getPose().position.x);
//        telemetry.addData("Y Position", drive.getPose().position.y);
//        telemetry.addData("Heading", drive.getPose().heading.toDouble());
//        telemetry.addLine("MECHANISMS");
//        telemetry.addData("Outtake Motor 1 Direction", outtakeMotor1.getDirection());
//        telemetry.addData("Outtake Motor 1 Power", outtakeMotor1.getPower());
//        telemetry.addData("Outtake Motor 1 Velocity", outtakeMotor1.getVelocity());
//        telemetry.addData("Outtake Motor 2 Direction", outtakeMotor2.getDirection());
//        telemetry.addData("Outtake Motor 2 Power", outtakeMotor2.getPower());
//        telemetry.addData("Outtake Motor 2 Velocity", outtakeMotor2.getVelocity());
//        telemetry.addLine("Toggles");
//        telemetry.addData("Mode", adjustable ? "Adjustable Mode" : "Standard Mode");
//        telemetry.addData("Location", farOrClose ? "Far Mode" : "Close Mode");
//        telemetry.addData("Servo : ", servoOn ? "On" : "Off");
//        telemetry.update();
    }
}
