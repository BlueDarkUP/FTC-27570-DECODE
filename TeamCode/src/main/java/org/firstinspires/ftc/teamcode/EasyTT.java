package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.vision.Class.ColorClassifier;

@TeleOp(name = "TeleOpMain11/20", group = "Competition")
public class EasyTT extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBehindDrive = null;
    private DcMotor rightBehindDrive = null;

    private DcMotor intakeMotor = null;
    private CRServo holdServo = null;
    private CRServo classifyServo = null;

    private ColorClassifier vision = new ColorClassifier();
    private boolean visionInitialized = false;

    private boolean isIntakeActive = false;
    private boolean lastLeftBumper = false;
    private IMU imu = null;

    private double lastDetectedDirection = -1.0;
    private boolean hasDetectedAnyColor = false;

    @Override
    public void runOpMode() {
        // 1. 硬件映射
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBehindDrive = hardwareMap.get(DcMotor.class, "LeftBehindDrive");
        rightBehindDrive = hardwareMap.get(DcMotor.class, "RightBehindDrive");

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        holdServo = hardwareMap.get(CRServo.class, "Hold");
        classifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBehindDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBehindDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBehindDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBehindDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        try {
            vision.init(hardwareMap, "ClassifyCam", telemetry);
            visionInitialized = true;
            telemetry.addLine("Vision initialized successfully.");
        } catch (Exception e) {
            visionInitialized = false;
            telemetry.addData("Vision Error", "Init Failed: " + e.getMessage());
        }

        telemetry.addLine("机器人已初始化。按 'Start' 开始。");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            vision.close();
            return;
        }

        while (opModeIsActive()) {

            double rawY = -gamepad1.left_stick_y;
            double rawX = gamepad1.left_stick_x;
            double rawRx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = Math.pow(rawY, 3);
            double x = Math.pow(rawX, 3);
            double rx = Math.pow(rawRx, 3);

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontDrive.setPower((rotY + rotX + rx) / denominator);
            leftBehindDrive.setPower((rotY - rotX + rx) / denominator);
            rightFrontDrive.setPower((rotY - rotX - rx) / denominator);
            rightBehindDrive.setPower((rotY + rotX - rx) / denominator);

            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumper) {
                isIntakeActive = !isIntakeActive;
            }
            lastLeftBumper = currentLeftBumper;

            if (isIntakeActive) {
                intakeMotor.setPower(1.0);
                holdServo.setPower(1.0);

                if (!visionInitialized) {
                    classifyServo.setPower(-1.0);
                    telemetry.addData("Classify", "Cam Failed -> Default -1.0");
                } else {
                    ColorClassifier.DetectionResult result = vision.getResult();

                    if (result == ColorClassifier.DetectionResult.GREEN) {
                        classifyServo.setPower(1.0);
                        lastDetectedDirection = 1.0;
                        hasDetectedAnyColor = true;
                        telemetry.addData("Classify", "GREEN -> 1.0");
                    }
                    else if (result == ColorClassifier.DetectionResult.PURPLE) {
                        classifyServo.setPower(-1.0);
                        lastDetectedDirection = -1.0;
                        hasDetectedAnyColor = true;
                        telemetry.addData("Classify", "PURPLE -> -1.0");
                    }
                    else {
                        if (hasDetectedAnyColor) {
                            double holdPower = lastDetectedDirection * 0.5;
                            classifyServo.setPower(holdPower);
                            telemetry.addData("Classify", "NONE -> Hold %.2f", holdPower);
                        } else {
                            classifyServo.setPower(-1.0);
                            telemetry.addData("Classify", "NONE (Startup) -> -1.0");
                        }
                    }
                }
            } else {
                intakeMotor.setPower(0.0);
                holdServo.setPower(0.0);
                classifyServo.setPower(0.0);
            }

            telemetry.addData("Intake Mode", isIntakeActive ? "ON" : "OFF");
            if(visionInitialized) {
                telemetry.addData("Vision Debug", vision.getDebugInfo());
            }
            telemetry.update();
        }

        vision.close();
    }
}