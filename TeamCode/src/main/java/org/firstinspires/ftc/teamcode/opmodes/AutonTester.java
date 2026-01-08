package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodes.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous
@Config
public class AutonTester extends OpMode {
    private Follower follower;
    private ElapsedTime waitTimer = new ElapsedTime();

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    DcMotorImplEx intakeMotor;
    DcMotorImplEx outtakeMotor;
    CRServoImplEx servo1;
    private final Pose startPose = new Pose(84, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(88, 12, Math.toRadians(70)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose leavePose = new Pose (88, 30, Math.toRadians(90));

    private final Pose pickup1Pose = new Pose(96, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pose1End = new Pose(140,84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(37, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose pose2End = new Pose(8, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(37, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose pose3End = new Pose(8, 36, Math.toRadians(0));
    private Path scorePreload;
    private PathChain grabPickup1, grab1, scorePickup1, grabPickup2, grab2, scorePickup2, grabPickup3, grab3, scorePickup3, leave;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose,pose1End))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(),pose1End.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pose1End, scorePose))
                .setLinearHeadingInterpolation(pose1End.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose,pose2End))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(),pose2End.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pose2End, scorePose))
                .setLinearHeadingInterpolation(pose2End.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,pose3End))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pose3End.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                waitTimer.reset();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    outtakeMotor.setVelocity(1720);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (outtakeMotor.getVelocity() > 1720) {
                        intakeMotor.setPower(0.5);
                        servo1.setPower(.3);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 15) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1);
                    setPathState(3);

                }
                break;

            case 3:
                intakeMotor.setPower(0.67);
                if(!follower.isBusy()) {
                    follower.followPath(grab1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy())
                {
                    intakeMotor.setPower(0.0);
                }
                else
                {
                    follower.followPath(scorePickup1);
                    setPathState(5);

                }
                break;

            case 5:
                waitTimer.reset();
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    outtakeMotor.setVelocity(1720);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (outtakeMotor.getVelocity() > 1720) {
                        intakeMotor.setPower(0.5);
                        servo1.setPower(.3);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 15) {
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(leave,true);
//                    setPathState(3);
                }
                break;

            case 7:

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorImplEx.class, "outtakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servo1 = hardwareMap.get(CRServoImplEx.class, "servo1");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Outtake Motor Velocity", outtakeMotor.getVelocity());
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    // Blue AprilTag is 20, Red AprilTag is 24
}