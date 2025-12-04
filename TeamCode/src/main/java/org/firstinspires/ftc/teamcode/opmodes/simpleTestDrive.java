package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class simpleTestDrive extends OpMode {
    MecanumDrive drive;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    }

    @Override
    public void loop() {
        double x = -gamepad1.left_stick_y * 0.35;
        double y = -gamepad1.left_stick_x * 0.35;
        double turn = -gamepad1.right_stick_x * 0.35;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), turn));
        drive.updatePoseEstimate();

        telemetry.addData("X Position", drive.getPose().position.x);
        telemetry.addData("Y Position", drive.getPose().position.y);
        telemetry.addData("Heading", drive.getPose().heading.toDouble());
        telemetry.update();
    }
}
