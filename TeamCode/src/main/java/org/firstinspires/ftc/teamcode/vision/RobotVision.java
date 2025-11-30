    package org.firstinspires.ftc.teamcode.vision;

    import android.util.Size;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import org.firstinspires.ftc.robotcore.external.ClassFactory;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
    import org.firstinspires.ftc.robotcore.external.navigation.Position;
    import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
    import org.firstinspires.ftc.teamcode.vision.Class.ColorBlobProcessor;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
    import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
    import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

    import java.util.List;

    public class RobotVision {

        // === 定义两个摄像头 ===
        private WebcamName webcamTag;
        private WebcamName webcamColor;

        private VisionPortal visionPortal;
        private AprilTagProcessor aprilTagProcessor;
        private ColorBlobProcessor colorProcessor;

        // AprilTag 参数
        private static final Position cameraPosition = new Position(DistanceUnit.CM, 0, 11.1, 44, 0);
        private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -72.5, 0, 0);
        private static final double HALF_FIELD_MM = 72 * 25.4;

        // Decimation 逻辑参数
        private float currentDecimation = 1.0f;
        private static final double K_RED = 0.11198037606120845;
        private static final double K_BLUE = 0.6998773503825528;

        public void init(HardwareMap hardwareMap) {
            webcamTag = hardwareMap.get(WebcamName.class, "Webcam 1");
            webcamColor = hardwareMap.get(WebcamName.class, "ClassifyCam");

            CameraName switchableCamera = ClassFactory.getInstance()
                    .getCameraManager().nameForSwitchableCamera(webcamTag, webcamColor);

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                    .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                    .setNumThreads(4)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .build();
            aprilTagProcessor.setDecimation(1);

            colorProcessor = new ColorBlobProcessor();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .setCameraResolution(new Size(640, 480))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTagProcessor)
                    .addProcessor(colorProcessor)
                    .build();

            switchToAprilTagMode();
        }

        public void switchToAprilTagMode() {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                if (visionPortal.getActiveCamera() != webcamTag) {
                    visionPortal.setActiveCamera(webcamTag);
                }
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                visionPortal.setProcessorEnabled(colorProcessor, false);
            }
        }

        public void switchToColorMode() {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                if (visionPortal.getActiveCamera() != webcamColor) {
                    visionPortal.setActiveCamera(webcamColor);
                }
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                visionPortal.setProcessorEnabled(colorProcessor, true);
            }
        }

        public void updateDecimationByNormalizedPos(double normX, double normY) {
            double d1 = (normX * normX) + Math.pow(1.0 - normY, 2);
            double d2 = Math.pow(1.0 - normX, 2) + Math.pow(1.0 - normY, 2);

            float newDecimation;
            if (d1 < K_RED || d2 < K_RED) newDecimation = 3.0f;
            else if (d1 <= K_BLUE || d2 <= K_BLUE) newDecimation = 2.0f;
            else newDecimation = 1.0f;

            if (currentDecimation != newDecimation) {
                aprilTagProcessor.setDecimation(newDecimation);
                currentDecimation = newDecimation;
            }
        }

        // === 功能1：全场定位 (严格排除 Spike Mark Tag) ===
        public Pose2D getRobotPose() {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                // 过滤掉 Spike Mark Tag (21, 22, 23) 确保不用于定位
                if (detection.id >= 21 && detection.id <= 23) continue;

                if (detection.metadata != null && detection.robotPose != null) {
                    double atY = detection.robotPose.getPosition().y;
                    double atX = detection.robotPose.getPosition().x;
                    double atYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    double finalX = atY + HALF_FIELD_MM;
                    double finalY = -atX + HALF_FIELD_MM;
                    double finalHeading = atYaw - 90;

                    return new Pose2D(DistanceUnit.MM, finalX, finalY, AngleUnit.DEGREES, finalHeading);
                }
            }
            return null;
        }

        // === 功能2：【新增】专门获取 Spike Mark ID ===
        public int getDecodeMarkId() {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                // 只寻找 21, 22, 23
                if (detection.id >= 21 && detection.id <= 23) {
                    return detection.id;
                }
            }
            return -1; // 未检测到
        }

        public ColorBlobProcessor.DetectionResult getColorResult() {
            return colorProcessor.getResult();
        }

        public void close() {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }