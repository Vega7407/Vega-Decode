package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.nextftc.hardware.impl.ServoEx;

@TeleOp
public class BigRobotDrove extends LinearOpMode {
    DcMotorEx motor1;
    ServoImplEx servo1;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");

        waitForStart();


    }
}
