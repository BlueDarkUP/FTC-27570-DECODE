package org.firstinspires.ftc.teamcode.vision.Class;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ColorBlobProcessor implements VisionProcessor {

    public enum DetectionResult {
        GREEN,
        PURPLE,
        NONE
    }

    // 核心变量
    private volatile DetectionResult result = DetectionResult.NONE;
    private volatile int greenCount = 0;
    private volatile int purpleCount = 0;

    // 绘图与矩阵缓存 (避免内存泄漏)
    private Mat tinyMat = new Mat();
    private Mat ycrcb = new Mat();
    private Mat mask = new Mat();
    private Size tinySize = new Size(80, 60);

    // 阈值 (保持你原来的数值)
    private Scalar greenLower = new Scalar(0, 0, 0);
    private Scalar greenUpper = new Scalar(255, 110, 150);
    private Scalar purpleLower = new Scalar(0, 140, 130);
    private Scalar purpleUpper = new Scalar(255, 255, 255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // 初始化代码，如果有的话
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // 这里完全复制你原来的 processFrame 逻辑
        Imgproc.resize(input, tinyMat, tinySize, 0, 0, Imgproc.INTER_NEAREST);
        Imgproc.cvtColor(tinyMat, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(ycrcb, greenLower, greenUpper, mask);
        greenCount = Core.countNonZero(mask);

        Core.inRange(ycrcb, purpleLower, purpleUpper, mask);
        purpleCount = Core.countNonZero(mask);

        int minThreshold = 180;

        if (greenCount > purpleCount && greenCount > minThreshold) {
            result = DetectionResult.GREEN;
        } else if (purpleCount > greenCount && purpleCount > minThreshold) {
            result = DetectionResult.PURPLE;
        } else {
            result = DetectionResult.NONE;
        }

        // 返回结果对象供 onDrawFrame 使用（如果需要屏幕画图）
        return result;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public DetectionResult getResult() {
        return result;
    }

    public String getDebugInfo() {
        return "G:" + greenCount + " P:" + purpleCount;
    }
}