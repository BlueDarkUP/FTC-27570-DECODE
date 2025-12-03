package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// 请确保这里的包名已经修正为 Processors 而不是 Class
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

    // === 定义两个独立的 VisionPortal ===
    private VisionPortal visionPortalTag;
    private VisionPortal visionPortalColor;

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

        // 1. 初始化 AprilTag 处理器
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setNumThreads(4)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        aprilTagProcessor.setDecimation(1);

        // 2. 初始化 颜色 处理器
        colorProcessor = new ColorBlobProcessor();

        // 3. 创建屏幕分割 ID (这样可以在屏幕上同时显示两个预览，或者你可以不加这一步)
        // 将屏幕水平分割：上方显示 Tag，下方显示 Color
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // === 构建 Tag 专用 Portal (MJPEG) ===
        visionPortalTag = new VisionPortal.Builder()
                .setCamera(webcamTag)
                .setLiveViewContainerId(viewIds[0]) // 使用第一个视图区域
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Tag 相机使用 MJPEG
                .addProcessor(aprilTagProcessor)
                .build();

        // === 构建 Color 专用 Portal (YUY2) ===
        visionPortalColor = new VisionPortal.Builder()
                .setCamera(webcamColor)
                .setLiveViewContainerId(viewIds[1]) // 使用第二个视图区域
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2) // Color 相机使用 YUY2 (避免兼容性问题)
                .addProcessor(colorProcessor)
                .build();

        // 为了防止 USB 带宽爆炸，初始化后默认先暂停 Color 相机的流
        // 默认进入 AprilTag 模式
        switchToAprilTagMode();
    }

    /**
     * 切换到 AprilTag 模式
     * 策略：开启 Tag 相机流，暂停 Color 相机流
     */
    public void switchToAprilTagMode() {
        // 先停掉不需要的，释放带宽
        if (visionPortalColor.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortalColor.stopStreaming();
        }

        // 再开启需要的
        if (visionPortalTag.getCameraState() != VisionPortal.CameraState.STREAMING) {
            visionPortalTag.resumeStreaming();
        }
    }

    /**
     * 切换到 颜色识别 模式
     * 策略：开启 Color 相机流，暂停 Tag 相机流
     */
    public void switchToColorMode() {
        // 先停掉不需要的
        if (visionPortalTag.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortalTag.stopStreaming();
        }

        // 再开启需要的
        if (visionPortalColor.getCameraState() != VisionPortal.CameraState.STREAMING) {
            visionPortalColor.resumeStreaming();
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

    // === 功能1：全场定位 ===
    public Pose2D getRobotPose() {
        // 只有当 Tag Portal 正在运行时才获取数据，否则可能数据陈旧
        if (visionPortalTag.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return null;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
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

    // === 功能2：Spike Mark ID ===
    public int getDecodeMarkId() {
        // 同样建议检查状态
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id >= 21 && detection.id <= 23) {
                return detection.id;
            }
        }
        return -1;
    }

    public ColorBlobProcessor.DetectionResult getColorResult() {
        // 即使流暂停了，Processor 可能会保留最后一帧的结果，但为了安全最好确认状态
        return colorProcessor.getResult();
    }

    public void close() {
        if (visionPortalTag != null) {
            visionPortalTag.close();
        }
        if (visionPortalColor != null) {
            visionPortalColor.close();
        }
    }
}