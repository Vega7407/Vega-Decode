package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class OutreachTeleOp extends OpMode {
    MecanumDrive drive;
    DcMotorImplEx intakeMotor;
    DcMotorImplEx outtakeMotor;
    ServoImplEx servo1;
    double motorPower;
    boolean servoOn;
    boolean servoDirection;
    double motorVelocity;
    double output;
    public static int target;
    public static double p = -0.002, i = 0, d = 0.0001, f = 0.01;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorImplEx.class, "outtakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");
        servoOn = false;
        output = 0;
        motorVelocity = 1400;

        servoDirection = false;
        motorPower = .5;
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
            if (outtakeMotor.getPower()<=.88) {
                motorPower+=.01;
                motorVelocity+= 50;
            }
        }
//        if (gamepad1.y) {
//            if (servoOn) {
//                outtakeMotor.setVelocity(1800);
//            } else {
//                outtakeMotor.setVelocity(0);
//            }
//            servoOn = !servoOn;
//        }
        if(gamepad1.bWasReleased())
        {
            if (outtakeMotor.getPower()>=.2) {
                motorPower-=.01;
                motorVelocity -= 50;
            }
        }
        if (gamepad1.right_stick_button) {
            outtakeMotor.setVelocity(motorVelocity);
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

//        if (gamepad1.left_bumper) {
//            if (intakeMotor.getDirection() == DcMotorSimple.Direction.FORWARD){
//                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            } else {
//                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//            }
//        }
//
//        if (gamepad1.right_bumper) {
//            if (outtakeMotor.getDirection() == DcMotorSimple.Direction.FORWARD){
//                outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            } else {
//                outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//            }
//        }

        double y = -gamepad1.left_stick_y * .5 ; // negate to make outtake front
        double x = -gamepad1.left_stick_x * .5; // negate to make outtake front
        double rx = -gamepad1.right_stick_x * .5;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(y, x), rx));
        drive.updatePoseEstimate();

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
