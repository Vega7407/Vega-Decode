package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class ConceptScanMotif extends OpMode {
    AprilTagProcessor processor;
    VisionPortal portal;

    // 21 - GPP, 22 - PGP, 23 - PPG
    public enum Motif {
        GPP(21),
        PGP(22),
        PPG(23);

        public final int id;

        Motif(int id) {
            this.id = id;
        }

        public static Motif fromId(int id) {
            for (Motif motif : Motif.values()) {
                if (motif.id == id) {
                    return motif;
                }
            }
            return null;
        }
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                processor
        );
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = processor.getDetections();
        if (!detections.isEmpty()) {
            Motif motif = Motif.fromId(detections.get(0).id);
            if (motif != null) {
                telemetry.addData("Motif", motif.name());
            }
        } else {
            telemetry.addData("Motif", "None");
        }
    }
}
