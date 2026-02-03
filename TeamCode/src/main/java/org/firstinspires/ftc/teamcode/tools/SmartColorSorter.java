package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "FinalColorSorterFinal", group = "Competition")
public class SmartColorSorter extends LinearOpMode {

    public enum DetectionResult {
        GREEN, PURPLE, NONE
    }

    private enum EjectState {
        IDLE, WAITING_FOR_OPEN, WAITING_FOR_FINISH
    }

    private OpenCvWebcam webcam;
    private ColorPipeline pipeline;

    private CRServo holdServo;
    private Servo gateServo;

    private DcMotor mozartMotor;
    private DcMotorEx intakeMotor;
    private DigitalChannel bigjuju;

    private List<DetectionResult> targetSequence;
    private int currentIndex = 0;
    private DetectionResult heldBall = DetectionResult.NONE;

    private long actionEndTime = 0;
    private long recoveryEndTime = 0;
    private final int ACTION_DELAY = 1000;
    private final int RECOVERY_DELAY = 800;

    private final double GATE_CLOSE = 0.82;
    private final double GATE_OPEN = 0.23;

    private EjectState ejectState = EjectState.IDLE;
    private long ejectTimer = 0;

    @Override
    public void runOpMode() {
        holdServo = hardwareMap.get(CRServo.class, "Hold");
        gateServo = hardwareMap.get(Servo.class, "bbb");
        mozartMotor = hardwareMap.get(DcMotor.class, "MOZART");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        bigjuju = hardwareMap.get(DigitalChannel.class, "bigjuju");
        bigjuju.setMode(DigitalChannel.Mode.INPUT);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mozartMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mozartMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gateServo.setPosition(GATE_CLOSE);
        targetSequence = new ArrayList<>(Arrays.asList(DetectionResult.PURPLE, DetectionResult.GREEN, DetectionResult.PURPLE));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "ClassifyCam"), cameraMonitorViewId);
        pipeline = new ColorPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        pipeline.setEnabled(true);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        intakeMotor.setPower(0.65);

        while (opModeIsActive()) {
            checkGamepadInput();
            long currentTime = System.currentTimeMillis();
            boolean objectDetected = bigjuju.getState();

            if (objectDetected && mozartMotor.getPower() > 0) {
                mozartMotor.setPower(0.0);
                telemetry.addData("SAFETY", "JUJU Triggered -> BRAKE");
            }

            if (ejectState == EjectState.WAITING_FOR_OPEN) {
                if (currentTime - ejectTimer > 300) {
                    gateServo.setPosition(GATE_CLOSE);
                    ejectState = EjectState.WAITING_FOR_FINISH;
                    ejectTimer = currentTime;
                }
            } else if (ejectState == EjectState.WAITING_FOR_FINISH) {
                if (currentTime - ejectTimer > 150) {
                    ejectState = EjectState.IDLE;
                }
            }

            if (currentTime < actionEndTime) {
                telemetry.addData("Status", "Acting (Reverse/Pass)...");
                updateTelemetry();
                continue;
            }

            if (currentTime < recoveryEndTime) {
                telemetry.addData("Status", "Recovering (Pushing Back)...");

                intakeMotor.setPower(0.0);

                if (!objectDetected) {
                    mozartMotor.setPower(0.6);
                }

                updateTelemetry();
                continue;
            }

            DetectionResult inputColor = pipeline.getAnalysis();

            if (currentIndex >= targetSequence.size()) {
                if (!objectDetected) mozartMotor.setPower(0.6);
                else mozartMotor.setPower(0.0);

                holdServo.setPower(-1.0);
                intakeMotor.setPower(1.0);
                telemetry.addData("Status", "Sequence Complete!");
            }
            else if (inputColor == DetectionResult.NONE) {
                telemetry.addData("Status", "Scanning...");
                if (objectDetected) mozartMotor.setPower(0.0);
                else mozartMotor.setPower(0.35);

                intakeMotor.setPower(0.65);

                if (heldBall == DetectionResult.NONE) holdServo.setPower(0);
                else holdServo.setPower(1.0);
            }
            else {
                int ballsInBucket = currentIndex;
                int ballsInHold = (heldBall != DetectionResult.NONE) ? 1 : 0;
                if (ballsInBucket + ballsInHold + 1 > 3) {
                    if (heldBall != DetectionResult.NONE) {
                        triggerSmartEject();
                        heldBall = DetectionResult.NONE;
                    }
                }

                DetectionResult neededColor = targetSequence.get(currentIndex);

                if (inputColor == neededColor) {
                    telemetry.addData("Action", "MATCH -> Pass");

                    boolean dropHeldBall = false;
                    if (heldBall != DetectionResult.NONE && (currentIndex + 1 < targetSequence.size())) {
                        if (heldBall == targetSequence.get(currentIndex + 1)) dropHeldBall = true;
                    }

                    if (dropHeldBall) {
                        holdServo.setPower(-1.0);
                        currentIndex += 2;
                        heldBall = DetectionResult.NONE;
                    } else {
                        holdServo.setPower(1.0);
                        currentIndex++;
                    }

                    mozartMotor.setPower(0.6);
                    intakeMotor.setPower(1.0);

                    actionEndTime = currentTime + ACTION_DELAY;
                }
                else {
                    intakeMotor.setPower(0.0);
                    mozartMotor.setPower(-0.65);

                    if (heldBall == DetectionResult.NONE) {
                        telemetry.addData("Action", "WRONG -> Lift");
                        gateServo.setPosition(GATE_CLOSE);
                        holdServo.setPower(1.0);
                        heldBall = inputColor;
                    }
                    else {
                        telemetry.addData("Action", "WRONG -> SWAP");
                        triggerSmartEject();
                        mozartMotor.setPower(-0.65);
                        intakeMotor.setPower(0.0);
                        holdServo.setPower(1.0);
                        heldBall = inputColor;
                    }

                    actionEndTime = currentTime + ACTION_DELAY;
                    recoveryEndTime = actionEndTime + RECOVERY_DELAY;
                }
            }
            updateTelemetry();
        }
        webcam.stopStreaming();
    }

    private void triggerSmartEject() {
        gateServo.setPosition(GATE_OPEN);
        intakeMotor.setPower(1.0);
        ejectState = EjectState.WAITING_FOR_OPEN;
        ejectTimer = System.currentTimeMillis();
    }

    private void checkGamepadInput() {
        if (gamepad1.a) { targetSequence = Arrays.asList(DetectionResult.PURPLE, DetectionResult.GREEN, DetectionResult.PURPLE); resetSequence(); }
        else if (gamepad1.b) { targetSequence = Arrays.asList(DetectionResult.GREEN, DetectionResult.PURPLE, DetectionResult.PURPLE); resetSequence(); }
        else if (gamepad1.x) { targetSequence = Arrays.asList(DetectionResult.PURPLE, DetectionResult.PURPLE, DetectionResult.GREEN); resetSequence(); }
    }

    private void resetSequence() {
        currentIndex = 0;
        heldBall = DetectionResult.NONE;
        gateServo.setPosition(GATE_CLOSE);
        actionEndTime = 0;
        recoveryEndTime = 0;
        ejectState = EjectState.IDLE;
    }

    private void updateTelemetry() {
        telemetry.addData("Index", currentIndex);
        telemetry.addData("Held", heldBall);
        telemetry.addData("Mozart", mozartMotor.getPower());
        telemetry.update();
    }

    public static class ColorPipeline extends OpenCvPipeline {
        private volatile boolean isEnabled = false;
        private volatile DetectionResult result = DetectionResult.NONE;
        Mat tinyMat = new Mat();
        Mat ycrcb = new Mat();
        Mat mask = new Mat();
        Size tinySize = new Size(60, 60);
        Scalar greenLower = new Scalar(0, 0, 0);
        Scalar greenUpper = new Scalar(255, 110, 150);
        Scalar purpleLower = new Scalar(0, 140, 130);
        Scalar purpleUpper = new Scalar(255, 255, 255);
        public void setEnabled(boolean enabled) { isEnabled = enabled; }
        @Override
        public Mat processFrame(Mat input) {
            if (!isEnabled) return input;
            Imgproc.resize(input, tinyMat, tinySize, 0, 0, Imgproc.INTER_NEAREST);
            Imgproc.cvtColor(tinyMat, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(ycrcb, greenLower, greenUpper, mask);
            int greenCount = Core.countNonZero(mask);
            Core.inRange(ycrcb, purpleLower, purpleUpper, mask);
            int purpleCount = Core.countNonZero(mask);
            int minThreshold = 250;
            if (greenCount > purpleCount && greenCount > minThreshold) result = DetectionResult.GREEN;
            else if (purpleCount > greenCount && purpleCount > minThreshold) result = DetectionResult.PURPLE;
            else result = DetectionResult.NONE;
            return input;
        }
        public DetectionResult getAnalysis() { return result; }
    }
}