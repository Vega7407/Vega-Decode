package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class testServoAndMotor extends OpMode {
    DcMotorImplEx motor1;
    ServoImplEx servo1;
    boolean motorOn;
    boolean servoOn;
    boolean servoDirection;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorImplEx.class, "motor1");
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");
        motorOn = false;
        servoOn = false;
        servoDirection = false;
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            if (motorOn) {
                motor1.setPower(0);
            } else {
                motor1.setPower(1);
            }
            motorOn = !motorOn;
        }

        if (gamepad1.b) {
            if (servoOn) {
                servo1.setPosition(0);
            } else {
                servo1.setPosition(1);
            }
            servoOn = !servoOn;
        }

        if (gamepad1.x) {
            if (motor1.getDirection() == DcMotorSimple.Direction.FORWARD){
                motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        if (gamepad1.y) {
            if (servo1.getDirection() == ServoImplEx.Direction.FORWARD){
                servo1.setDirection(ServoImplEx.Direction.REVERSE);
            } else {
                servo1. setDirection(ServoImplEx.Direction.FORWARD);
            }
        }

        telemetry.addData("Motor Direction", motor1.getDirection());                
        telemetry.addData("Servo Power", servo1.getDirection());
        telemetry.update();
    }
}
