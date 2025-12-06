package org.firstinspires.ftc.teamcode.opmodes;

import android.service.controls.Control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.impl.MotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;


@TeleOp
@Config
public class MainTeleOp extends OpMode {
    MecanumDrive drive;
    DcMotorImplEx intakeMotor;
    DcMotorImplEx outtakeMotor;
    MotorEx outtakeMotorEx;
    ServoImplEx servo1;
    ControlSystem controller;
    double motorPower;
    boolean servoOn;
    boolean servoDirection;
    double output;
    public static int target;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.01, 0.01, 0.01);
    public static double kS, kV, kA;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorImplEx.class, "outtakeMotor");
        outtakeMotorEx = new MotorEx(outtakeMotor);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");
        servoOn = false;
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

        if (gamepad1.right_trigger > 0.1) {
            outtakeMotor.setPower(motorPower);
        } else {
            outtakeMotor.setPower(0);
        }

        if(gamepad1.aWasReleased())
        {
            if (motorPower <=.99) {
                motorPower+=.01;
            }
        }

        if (gamepad1.y) {
            if (servoOn) {
                servo1.setPosition(1);
            } else {
                servo1.setPosition(0);
            }
            servoOn = !servoOn;
        }

        if(gamepad1.bWasReleased())
        {
            if (outtakeMotor.getPower()>=.1) {
                motorPower-=.01;
            }
        }

        if (gamepad1.left_trigger > 0.1) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0);
        }


        /*
        if(gamepad1.right_trigger >0){
            motor1.setPower(gamepad1.right_trigger);

        else(){
            motor1.set\Power(0);
        }
        */

        double y = -gamepad1.left_stick_y ; // negate to make outtake front
        double x = -gamepad1.left_stick_x; // negate to make outtake front
        double rx = -gamepad1.right_stick_x;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, x), rx));
        drive.updatePoseEstimate();

//        outtakeMotorEx.setPower(controller.calculate(outtakeMotorEx.getState()));

        telemetry.addData("X Position", drive.getPose().position.x);
        telemetry.addData("Y Position", drive.getPose().position.y);
        telemetry.addData("Heading", drive.getPose().heading.toDouble());
        telemetry.addData("Intake Motor Direction", intakeMotor.getDirection());
        telemetry.addData("Intake Motor Velocity", intakeMotor.getVelocity());
        telemetry.addData("Outtake Motor Direction", outtakeMotor.getDirection());
        telemetry.addData("Outtake Motor Power", outtakeMotor.getPower());
        telemetry.addData("Outtake Motor Velocity", outtakeMotor.getVelocity());
        telemetry.addData("Servo Power", servo1.getDirection());
        telemetry.update();
    }
}
