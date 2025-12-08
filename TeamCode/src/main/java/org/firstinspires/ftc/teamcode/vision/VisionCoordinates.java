package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

@TeleOp(name = "Color-Locator (Coordinates)", group = "Concept")
public class VisionCoordinates extends LinearOpMode
{
    // ==========================================
    // 关键配置：请根据你的机器人实际测量值修改此处
    // ==========================================
    static class CameraConfig {
        // 摄像头的安装高度 (单位: cm 或 inch，输出坐标将与此单位一致)
        static final double CAMERA_HEIGHT = 20.0;

        // 摄像头的俯仰角 (单位: 度)。
        // 0度 = 水平向前看
        // 90度 = 垂直向下看
        // 建议使用手机水平仪应用测量
        static final double CAMERA_PITCH = 30.0;

        // 摄像头的垂直视野 (Vertical FOV)
        // Logitech C270/C310 大约是 42度
        // Logitech C920 大约是 43度 (宽屏模式) 到 55度 (4:3模式)
        // Control Hub 内置摄像头参数各异，通常在 40-60之间
        static final double VERTICAL_FOV = 43.30;   // 垂直视野
        static final double HORIZONTAL_FOV = 70.42; // 水平视野

        static final int RES_WIDTH = 640;
        static final int RES_HEIGHT = 360;
    }

    @Override
    public void runOpMode()
    {
        // 1. 创建紫色识别器
        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // 2. 创建绿色识别器
        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // 3. 构建 VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .setCameraResolution(new Size(CameraConfig.RES_WIDTH, CameraConfig.RES_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("Status", "Calculating Real-World Coords");

            // 获取数据
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, purpleBlobs);

            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, greenBlobs);

            telemetry.addLine("--- PURPLE ---");
            processAndDisplayBlobs(purpleBlobs);

            telemetry.addLine("\n--- GREEN ---");
            processAndDisplayBlobs(greenBlobs);

            telemetry.update();
            sleep(50);
        }
    }

    /**
     * 处理 Blob 列表，计算物理坐标并显示
     */
    private void processAndDisplayBlobs(List<ColorBlobLocatorProcessor.Blob> blobs) {
        if (blobs.isEmpty()) {
            telemetry.addLine("  None");
            return;
        }

        for(ColorBlobLocatorProcessor.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();
            Point center = boxFit.center;

            // 调用核心计算函数
            Point realWorldPos = calculateGroundCoordinates(center.x, center.y);

            // 显示格式：
            // [X: 左右偏移] [Z: 前方距离]
            // X>0 在右边, X<0 在左边
            telemetry.addLine(String.format("  Scr:(%3d,%3d) -> Pos X:%5.1f  Z:%5.1f",
                    (int)center.x, (int)center.y,
                    realWorldPos.x, realWorldPos.y)); //这里 Point.y 存的是距离Z
        }
    }

    /**
     * 核心数学函数：将屏幕像素坐标转换为地面物理坐标
     * 假设地面平坦，摄像头高度和角度已知。
     *
     * @param screenX 像素 X (0 ~ Width)
     * @param screenY 像素 Y (0 ~ Height)
     * @return Point对象，其中 x=横向偏移(Lateral), y=前方距离(Forward/Z)
     */
    private Point calculateGroundCoordinates(double screenX, double screenY) {
        // 1. 将屏幕坐标转换为归一化坐标 (-1.0 到 1.0)
        // OpenCV坐标系：左上角是(0,0)，Y向下增加
        // 我们需要中心点是(0,0)
        double centerX = CameraConfig.RES_WIDTH / 2.0;
        double centerY = CameraConfig.RES_HEIGHT / 2.0;

        // 2. 计算每个像素对应的角度
        // 计算焦距 (以像素为单位)
        // focalLength = (Resolution / 2) / tan(FOV / 2)
        double focalLengthY = centerY / Math.tan(Math.toRadians(CameraConfig.VERTICAL_FOV / 2.0));
        double focalLengthX = centerX / Math.tan(Math.toRadians(CameraConfig.HORIZONTAL_FOV / 2.0));

        // 3. 计算物体在垂直方向相对于光轴的角度 (Pitch Offset)
        // 屏幕Y坐标越大，像素越靠下，代表角度越垂直向下 (角度变大)
        double pitchOffset = Math.atan((screenY - centerY) / focalLengthY);

        // 4. 计算物体在水平方向相对于光轴的角度 (Yaw Offset)
        double yawOffset = Math.atan((screenX - centerX) / focalLengthX);

        // 5. 计算实际物理距离 (Z轴 / Forward)
        // 总俯仰角 = 摄像头安装角度 + 像素偏角
        double totalPitch = Math.toRadians(CameraConfig.CAMERA_PITCH) + pitchOffset;

        // 如果角度太小(看天)或是负数，距离会无限大或无意义，做一个保护
        if (totalPitch <= 0.1) totalPitch = 0.1;

        // 距离 = 高度 / tan(总俯仰角)
        double distForward = CameraConfig.CAMERA_HEIGHT / Math.tan(totalPitch);

        // 6. 计算实际横向偏移 (X轴 / Lateral)
        // 偏移 = 前方距离 * tan(水平偏角)
        double distLateral = distForward * Math.tan(yawOffset);

        return new Point(distLateral, distForward);
    }
}