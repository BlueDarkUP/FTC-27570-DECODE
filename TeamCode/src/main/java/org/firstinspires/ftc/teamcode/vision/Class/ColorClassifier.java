package org.firstinspires.ftc.teamcode.vision.Class;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ColorClassifier {

    public enum DetectionResult {
        GREEN,
        PURPLE,
        NONE
    }

    private OpenCvWebcam webcam;
    private SoftwareResizePipeline pipeline;
    private Telemetry telemetry; // 可选，用于调试

    /**
     * 初始化相机
     * @param hardwareMap 硬件映射
     * @param webcamName 配置文件中摄像头的名字 (例如 "ClassifyCam")
     * @param telemetry 遥测对象，如果不需要调试可以传 null
     */
    public void init(HardwareMap hardwareMap, String webcamName, Telemetry telemetry) {
        this.telemetry = telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        pipeline = new SoftwareResizePipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                if (telemetry != null) {
                    telemetry.addData("Vision Error", "Camera open failed");
                    telemetry.update();
                }
            }
        });
    }

    public DetectionResult getResult() {
        if (pipeline == null) return DetectionResult.NONE;
        return pipeline.getAnalysis();
    }

    public String getDebugInfo() {
        if (pipeline == null) return "Pipeline not ready";
        return "G:" + pipeline.greenCount + " P:" + pipeline.purpleCount;
    }

    public void close() {
        if (webcam != null) {
            webcam.stopStreaming();
        }
    }

    private static class SoftwareResizePipeline extends OpenCvPipeline {
        public volatile DetectionResult result = DetectionResult.NONE;
        public volatile int greenCount = 0;
        public volatile int purpleCount = 0;

        Mat tinyMat = new Mat();
        Mat ycrcb = new Mat();
        Mat mask = new Mat();

        Size tinySize = new Size(80, 60);

        // 阈值设定
        Scalar greenLower = new Scalar(0, 0, 0);
        Scalar greenUpper = new Scalar(255, 110, 150);
        Scalar purpleLower = new Scalar(0, 140, 130);
        Scalar purpleUpper = new Scalar(255, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.resize(input, tinyMat, tinySize, 0, 0, Imgproc.INTER_NEAREST);

            Imgproc.cvtColor(tinyMat, ycrcb, Imgproc.COLOR_RGB2YCrCb);

            Core.inRange(ycrcb, greenLower, greenUpper, mask);
            greenCount = Core.countNonZero(mask);

            Core.inRange(ycrcb, purpleLower, purpleUpper, mask);
            purpleCount = Core.countNonZero(mask);

            int minThreshold = 240; // 最小像素阈值

            if (greenCount > purpleCount && greenCount > minThreshold) {
                result = DetectionResult.GREEN;
            } else if (purpleCount > greenCount && purpleCount > minThreshold) {
                result = DetectionResult.PURPLE;
            } else {
                result = DetectionResult.NONE;
            }

            return input;
        }

        public DetectionResult getAnalysis() {
            return result;
        }
    }
}