package org.firstinspires.ftc.teamcode.vision.QuickScope;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagLocalizer {
    private static final Position cameraPosition = new Position(DistanceUnit.CM,
            0, 11.1, 44, 0);
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -72.5, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double HALF_FIELD_MM = 72 * 25.4;

    private static final int SPIKE_MARK_ID_LEFT = 21;   // 返回 1
    private static final int SPIKE_MARK_ID_MIDDLE = 22; // 返回 2
    private static final int SPIKE_MARK_ID_RIGHT = 23;  // 返回 3

    private float currentDecimation = 1.0f;
    private static final double K_RED = 0.11198037606120845;
    private static final double K_BLUE = 0.6998773503825528;

    public AprilTagLocalizer(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setNumThreads(4)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
    }

    public void updateDecimationByNormalizedPos(double normX, double normY) {
        double d1 = (normX * normX) + Math.pow(1.0 - normY, 2);
        double d2 = Math.pow(1.0 - normX, 2) + Math.pow(1.0 - normY, 2);

        float newDecimation;

        if (d1 < K_RED || d2 < K_RED) {
            // 红色内部 -> 极近距离 (< 1.2m)
            newDecimation = 3.0f;
        } else if (d1 <= K_BLUE || d2 <= K_BLUE) {
            // 蓝红之间 -> 中距离 (1.2m - 3.0m)
            newDecimation = 2.0f;
        } else {
            // 蓝色弧线下方 -> 远距离 (> 3.0m)
            newDecimation = 1.0f;
        }

        // 4. 仅在状态改变时调用底层API，避免性能损耗
        if (currentDecimation != newDecimation) {
            aprilTag.setDecimation(newDecimation);
            currentDecimation = newDecimation;
        }
    }


    public Pose2D getRobotPose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == SPIKE_MARK_ID_LEFT ||
                    detection.id == SPIKE_MARK_ID_MIDDLE ||
                    detection.id == SPIKE_MARK_ID_RIGHT) {
                continue;
            }

            if (detection.metadata != null && detection.robotPose != null) {
                double aprilTagX = detection.robotPose.getPosition().x;
                double aprilTagY = detection.robotPose.getPosition().y;
                double aprilTagYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                double finalX_mm = aprilTagY + HALF_FIELD_MM;
                double finalY_mm = -aprilTagX + HALF_FIELD_MM;
                double finalHeading_deg = aprilTagYaw - 90;

                return new Pose2D(
                        DistanceUnit.MM,
                        finalX_mm,
                        finalY_mm,
                        AngleUnit.DEGREES,
                        finalHeading_deg
                );
            }
        }

        return null;
    }

    public int getSpikeMarkLocation() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                switch (detection.id) {
                    case SPIKE_MARK_ID_LEFT:
                        return 1;
                    case SPIKE_MARK_ID_MIDDLE:
                        return 2;
                    case SPIKE_MARK_ID_RIGHT:
                        return 3;
                }
            }
        }
        return 0;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}