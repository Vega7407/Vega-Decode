package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class testMotor extends OpMode {

    Motor motor;
    CRServoImplEx servo;
    Gamepad gp1;
    boolean motorOn;
    boolean servoOn;
    double servoPower;
    double motorPower;

    @Override
    public void init() {
        motor = new Motor(hardwareMap.get(DcMotorEx.class, "motor"));
        servo = hardwareMap.get(CRServoImplEx.class, "servo");
        gp1 = new Gamepad();
        motorOn = false;
        servoOn = false;
        motorPower = 1;
        servoPower = 1;
    }

    @Override
    public void loop() {
        if (gp1.a) {
            if (motorOn) {
                motor.setPower(0);
            } else {
                motor.setPower(motorPower);
            }
        }
        if (gp1.b) {
            if (servoOn) {
                servo.setPower(0);
            } else {
                servo.setPower(servoPower);
            }
        }
        if (gp1.x) {
            motor.reverse();
        }
        if (gp1.y) {
            servoPower *= -1;
        }
    }
}
