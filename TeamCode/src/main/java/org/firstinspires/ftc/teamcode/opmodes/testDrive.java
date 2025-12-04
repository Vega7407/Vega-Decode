package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class testDrive extends OpMode {

    private static final double NORMAL_SPEED = 0.75;
    private static final double SLOW_SPEED = 0.3;
    private static final double FAST_SPEED = 1;

    private static final double SLEW_XY_PER_S = 3.0;
    private static final double SLEW_TURN_PER_S = 4.0;

    private static final double LOCK_POWER = 0.28; // medium brace like you chose

    private double headingOffset = 0.0;
    private final SlewLimiter slewX = new SlewLimiter(SLEW_XY_PER_S);
    private final SlewLimiter slewY = new SlewLimiter(SLEW_XY_PER_S);
    private final SlewLimiter slewTurn = new SlewLimiter(SLEW_TURN_PER_S);

    MecanumDrive drive;
    ElapsedTime timer;
    double lastTime;
    boolean fieldCentric;

    // ===== NEW for LOCK =====
    private boolean lockActive = false;
    private boolean lockLatch = false;
    // ========================

    @Override
    public void init() {
        timer = new ElapsedTime();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        fieldCentric = true;
    }

    @Override
    public void loop() {

        // ========= LOCK TOGGLE ON GP2 DPAD DOWN =========
        if (gamepad2.dpad_down && !lockLatch) {
            lockActive = !lockActive;
            lockLatch = true;
            // reset slew so no jump when unlock
            slewX.reset();
            slewY.reset();
            slewTurn.reset();
        }
        if (!gamepad2.dpad_down) lockLatch = false;

        // ========= IF LOCKED — BRAKE & SKIP EVERYTHING =========
        if (lockActive) {
            // active X-brace pattern
            drive.setRawWheelPowers(+LOCK_POWER, -LOCK_POWER, +LOCK_POWER, -LOCK_POWER);
            drive.updatePoseEstimate(); // still keep localization alive
            telemetry.addData("LOCK MODE", "ACTIVE");
            telemetry.update();
            return; // <<< important
        }
        // ========================================================

        timer.reset();
        lastTime = timer.seconds();

        double now = timer.seconds();
        double dt = Math.max(1e-3, now - lastTime);
        lastTime = now;

        double scale = NORMAL_SPEED;
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            scale = SLOW_SPEED;
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            scale = FAST_SPEED;
        } else if (gamepad1.left_bumper && gamepad1.right_bumper) {
            scale = NORMAL_SPEED;
        }

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            fieldCentric = !fieldCentric;
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad2.dpad_up) {
            headingOffset = drive.getPose().heading.toDouble();
        }

        x *= scale;
        y *= scale;
        turn *= scale;

        if (fieldCentric) {
            double heading = drive.getPose().heading.toDouble() - headingOffset;
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);

            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;
            x = rotX;
            y = rotY;
        }

        x    = slewX.calculate(x, dt);
        y    = slewY.calculate(y, dt);
        turn = slewTurn.calculate(turn, dt);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), turn));
        drive.updatePoseEstimate();

        telemetry.addData("Drive Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("LOCK MODE", lockActive ? "ACTIVE" : "OFF");
        telemetry.update();
    }

    private static class SlewLimiter {
        private final double ratePerSec;
        private double last = 0;
        SlewLimiter(double ratePerSec) { this.ratePerSec = Math.max(1e-6, ratePerSec); }
        double calculate(double target, double dt) {
            double maxDelta = ratePerSec * dt;
            double delta = target - last;
            if (delta > maxDelta) delta = maxDelta;
            if (delta < -maxDelta) delta = -maxDelta;
            last += delta;
            return last;
        }
        void reset(){ last = 0; } // <-- needed for clean unlock
    }
}
