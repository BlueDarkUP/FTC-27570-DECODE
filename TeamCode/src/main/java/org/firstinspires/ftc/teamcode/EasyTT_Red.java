package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.vision.Class.ColorClassifier;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.IPoseProvider;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;
import org.firstinspires.ftc.teamcode.vision.QuickScope.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.vision.QuickScope.ArcherLogic;
import org.firstinspires.ftc.teamcode.vision.QuickScope.CalculationParams;
import org.firstinspires.ftc.teamcode.vision.QuickScope.LaunchSolution;

@TeleOp(name = "TeleOpRed", group = "Competition")
public class EasyTT_Red extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBehindDrive = null;
    private DcMotor rightBehindDrive = null;

    private DcMotor intakeMotor = null;

    private DcMotorEx shMotor = null;
    public static final double MOTOR_TICK_COUNT = 28.0;

    private CRServo holdServo = null;
    private CRServo classifyServo = null;
    private CRServo washer = null;

    private Servo servoRP = null;
    private Servo servoLP = null;

    private DcMotor mozart = null;
    private RevColorSensorV3 ballSensor = null;

    private ColorClassifier vision = new ColorClassifier();
    private boolean visionInitialized = false;
    private IMU imu = null;

    private boolean isIntakeActive = false;
    private boolean lastLeftBumper = false;

    private boolean isAimMode = false;
    private boolean lastRightBumper = false;

    private double lastDetectedDirection = -1.0;
    private boolean hasDetectedAnyColor = false;
    private boolean isMozartBraked = false;

    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private ArcherLogic archerLogic;
    private ElapsedTime runtime = new ElapsedTime();

    private double headingOffset = 0.0;
    private String targetAlliance = "Red";
    private double lastVisionUpdateTime = 0.0;

    private LaunchSolution currentSolution = null;
    private LaunchSolution lastValidSolution = null;

    private SimpleKalmanFilter angleFilter = new SimpleKalmanFilter(2.0, 1.0, 0.1);
    private double filteredLauncherAngle = 67.5;

    private double smoothSpeed = 0.0;
    private final double SPEED_FILTER_ALPHA = 0.7;

    // --- 视觉速度阈值设置 ---
    private static final double VISION_CORRECT_THRESHOLD = 0.05;
    private static final double VISION_DETECT_THRESHOLD = 0.5;

    private static final double MIN_ANGLE_DEG = 39.999999999998;
    private static final double MAX_ANGLE_DEG = 67.444444444448;

    private double targetHeading = 0.0;
    private double headingLastError = 0.0;
    private double lastPidTime = 0.0;

    private static final double P_TURN = 0.01;
    private static final double D_TURN = 0.0005;
    private static final double K_STATIC = 0.15;
    private static final double MAX_AUTO_TURN = 1;

    private static final double RPM_TOLERANCE = 50.0;

    private double lastRumbleTime = 0.0;
    private String visionStatus = "Ready";

    // --- 修改点：定义驾驶员操作修正角度 ---
    // 如果之前向前推是去 -90度，这里填 90.0 来修正回 0度。
    // 如果发现方向修正反了（变成去+90了），请将此值改为 -90.0
    private static final double DRIVER_ORIENTATION_CORRECTION = 90.0;

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBehindDrive = hardwareMap.get(DcMotor.class, "LeftBehindDrive");
        rightBehindDrive = hardwareMap.get(DcMotor.class, "RightBehindDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBehindDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBehindDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBehindDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBehindDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            shMotor = hardwareMap.get(DcMotorEx.class, "SH");
            shMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            PIDFCoefficients pidfNew = new PIDFCoefficients(250, 0.1, 30, 13);
            shMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
            shMotor.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Error", "SH Motor Not Found or Config Error");
        }

        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        holdServo = hardwareMap.get(CRServo.class, "Hold");
        classifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        washer = hardwareMap.get(CRServo.class, "washer");

        intakeMotor.setPower(0);
        holdServo.setPower(0);
        classifyServo.setPower(0);
        washer.setPower(0);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");

        servoRP.setPosition(0.91);
        servoLP.setPosition(0.078);

        try {
            mozart = hardwareMap.get(DcMotor.class, "MOZART");
            mozart.setDirection(DcMotor.Direction.FORWARD);
            mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mozart.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Warning", "MOZART Not Found");
        }

        try {
            ballSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        } catch (Exception e) {
            telemetry.addData("Warning", "Color Sensor Not Found");
        }

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuParameters);

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        pinpointPoseProvider.initialize();
        archerLogic = new ArcherLogic();

        try {
            vision.init(hardwareMap, "ClassifyCam", telemetry);
            visionInitialized = true;
            vision.setEnabled(false);
        } catch (Exception e) {
            visionInitialized = false;
            telemetry.addData("Vision Error", e.getMessage());
        }

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        try {
            if (isStopRequested()) return;

            runtime.reset();
            lastPidTime = runtime.seconds();
            performVisionCorrection(0.0);

            while (opModeIsActive()) {

                pinpointPoseProvider.update();

                double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
                double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);
                double normalizationX = robotX_cm / 365.76;
                double normalizationY = robotY_cm / 365.76;

                double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
                double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
                double rawSpeed = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);

                smoothSpeed = (SPEED_FILTER_ALPHA * rawSpeed) + ((1.0 - SPEED_FILTER_ALPHA) * smoothSpeed);

                if (smoothSpeed < 0.1) {
                    smoothSpeed = 0.0;
                }

                if (smoothSpeed < VISION_DETECT_THRESHOLD) {
                    boolean corrected = performVisionCorrection(smoothSpeed);
                    if (corrected) {
                        visionStatus = "Correcting (Active)";
                        if (runtime.seconds() - lastRumbleTime > 1.0) {
                            gamepad1.rumbleBlips(2);
                            lastRumbleTime = runtime.seconds();
                        }
                    } else {
                        visionStatus = "Monitoring (No Replace)";
                    }
                } else {
                    visionStatus = "Disabled (Too Fast)";
                }

                double direction_deg = Math.toDegrees(Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s));
                if (direction_deg < 0) direction_deg += 360;

                if (gamepad1.x) targetAlliance = "Blue";
                if (gamepad1.b) targetAlliance = "Red";
                if (gamepad1.a) {
                    pinpointPoseProvider.reset();
                    imu.resetYaw();
                    headingOffset = 0;
                }

                CalculationParams currentParams = new CalculationParams(
                        normalizationX, normalizationY, smoothSpeed, direction_deg, targetAlliance
                );

                LaunchSolution tempSolution = archerLogic.calculateSolution(currentParams);

                if (tempSolution != null) {
                    lastValidSolution = tempSolution;
                    currentSolution = tempSolution;
                } else {
                    currentSolution = lastValidSolution;
                }

                boolean currentRightBumper = gamepad1.right_bumper;
                if (currentRightBumper && !lastRightBumper) {
                    isAimMode = !isAimMode;
                    if (isAimMode) {
                        headingLastError = 0;
                        lastPidTime = runtime.seconds();
                        if (currentSolution == null) {
                            targetHeading = getRobotFieldHeading();
                        }
                    }
                }
                lastRightBumper = currentRightBumper;

                if (isAimMode && currentSolution != null) {
                    double rawHeadingDeg = currentSolution.aimAzimuthDeg + 180;
                    targetHeading = normalizeAngle(rawHeadingDeg);
                }

                double targetRPM = 1000.0;
                double currentRpm = 0.0;

                if (isAimMode && currentSolution != null) {
                    targetRPM = currentSolution.motorRpm;
                }

                if (shMotor != null) {
                    double currentVel = shMotor.getVelocity();
                    currentRpm = (currentVel * 60.0) / MOTOR_TICK_COUNT;

                    double targetVelocityTPS = (targetRPM * MOTOR_TICK_COUNT) / 60.0;
                    shMotor.setVelocity(targetVelocityTPS);
                }

                double rawTargetAngle = (currentSolution != null) ? currentSolution.launcherAngle : 67.5;
                rawTargetAngle = Range.clip(rawTargetAngle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

                filteredLauncherAngle = angleFilter.updateEstimate(rawTargetAngle);

                if (isAimMode) {
                    setLauncherServos(filteredLauncherAngle);

                    if (mozart != null) {
                        if (gamepad1.left_trigger > 0.05) {
                            double rpmError = Math.abs(targetRPM - currentRpm);

                            if (rpmError <= RPM_TOLERANCE) {
                                mozart.setPower(1.0);
                            } else {
                                mozart.setPower(0.0);
                            }
                        } else {
                            mozart.setPower(0.0);
                        }
                    }
                } else {
                    servoRP.setPosition(0.91);
                    servoLP.setPosition(0.078);
                }

                if (gamepad1.options) {
                    imu.resetYaw();
                    headingOffset = 0;
                }

                double rawY = -gamepad1.left_stick_y;
                double rawX = gamepad1.left_stick_x;
                double rotationPower;

                if (isAimMode) {
                    double currentTime = runtime.seconds();
                    double dt = currentTime - lastPidTime;
                    if (dt < 0.001) dt = 0.001;

                    double currentHeading = getRobotFieldHeading();
                    double headingError = normalizeAngle(currentHeading - targetHeading);

                    double pTerm = headingError * P_TURN;
                    double dTerm = (headingError - headingLastError) / dt * D_TURN;

                    double pidOut = pTerm + dTerm;

                    if (Math.abs(headingError) > 1.0) {
                        if (pidOut > 0) pidOut += K_STATIC;
                        else pidOut -= K_STATIC;
                    }

                    rotationPower = Range.clip(pidOut, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    headingLastError = headingError;
                    lastPidTime = currentTime;
                } else {
                    rotationPower = rx(gamepad1.right_stick_x);
                }

                // --- 关键修改开始 ---

                // 1. 获取真实场地角度（保持不变，给自瞄用）
                double botHeadingRad = Math.toRadians(getRobotFieldHeading());

                // 2. 计算仅用于手动驾驶的“修正后角度”
                // 原理：欺骗底盘算法，让它以为机器人的角度偏了90度，从而修正手柄输入的映射
                double drivingHeadingRad = botHeadingRad + Math.toRadians(DRIVER_ORIENTATION_CORRECTION);

                // 3. 在矢量旋转公式中使用修正后的 drivingHeadingRad
                double rotX = x(rawX) * Math.cos(-drivingHeadingRad) - y(rawY) * Math.sin(-drivingHeadingRad);
                double rotY = x(rawX) * Math.sin(-drivingHeadingRad) + y(rawY) * Math.cos(-drivingHeadingRad);

                // --- 关键修改结束 ---

                rotX = rotX * 1.1;
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationPower), 1);
                leftFrontDrive.setPower((rotY + rotX + rotationPower) / denominator);
                leftBehindDrive.setPower((rotY - rotX + rotationPower) / denominator);
                rightFrontDrive.setPower((rotY - rotX - rotationPower) / denominator);
                rightBehindDrive.setPower((rotY + rotX - rotationPower) / denominator);

                boolean currentLeftBumper = gamepad1.left_bumper;
                if (currentLeftBumper && !lastLeftBumper) {
                    isIntakeActive = !isIntakeActive;
                    if (isIntakeActive) {
                        if (visionInitialized) {
                            vision.setEnabled(true);
                        }
                        isMozartBraked = false;
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else {
                        if (visionInitialized) {
                            vision.setEnabled(false);
                        }
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }
                }
                lastLeftBumper = currentLeftBumper;

                if (isIntakeActive) {
                    intakeMotor.setPower(1.0);
                    washer.setPower(1.0);
                    holdServo.setPower(1.0);

                    if (!visionInitialized) {
                        classifyServo.setPower(1.0);
                    } else {
                        ColorClassifier.DetectionResult result = vision.getResult();
                        if (result == ColorClassifier.DetectionResult.GREEN) {
                            classifyServo.setPower(-1.0);
                            lastDetectedDirection = 1.0;
                            hasDetectedAnyColor = true;
                        } else if (result == ColorClassifier.DetectionResult.PURPLE) {
                            classifyServo.setPower(1.0);
                            lastDetectedDirection = -1.0;
                            hasDetectedAnyColor = true;
                        } else {
                            if (hasDetectedAnyColor) {
                                classifyServo.setPower(-lastDetectedDirection * 0.5);
                            } else {
                                classifyServo.setPower(1.0);
                            }
                        }
                    }

                    if (!isAimMode && mozart != null && ballSensor != null) {
                        if (!isMozartBraked) {
                            NormalizedRGBA colors = ballSensor.getNormalizedColors();
                            if (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1) {
                                isMozartBraked = true;
                            }
                        }
                        if (isMozartBraked) {
                            mozart.setPower(0.0);
                        } else {
                            mozart.setPower(1.0);
                        }
                    }

                } else {
                    intakeMotor.setPower(0.0);
                    holdServo.setPower(0.0);
                    classifyServo.setPower(0.0);
                    washer.setPower(0.0);

                    if (!isAimMode && mozart != null) mozart.setPower(0.0);
                }

                telemetry.addData("Mode", isAimMode ? "AIMING (RUN-N-GUN)" : "Manual");
                telemetry.addData("Vision Status", visionStatus);
                telemetry.addData("Driver Offset", "%.1f deg", DRIVER_ORIENTATION_CORRECTION); // 显示当前修正值

                if (shMotor != null) {
                    telemetry.addData("SH Target RPM", "%.1f", targetRPM);
                    telemetry.addData("SH Current RPM", "%.1f", currentRpm);
                    telemetry.addData("X","%.5f",normalizationX);
                    telemetry.addData("Y","%.5f",normalizationY);
                    if (currentSolution != null) {
                        telemetry.addData("Angle", "%.2f", currentSolution.launcherAngle);
                        telemetry.addData("Heading", "%.2f", currentSolution.aimAzimuthDeg);
                        telemetry.addData("RPM","%.2f",currentSolution.motorRpm);
                    } else {
                        telemetry.addLine(">> No Solution <<");
                    }
                    if (isAimMode && gamepad1.left_trigger > 0.05) {
                        double diff = Math.abs(targetRPM - currentRpm);
                        telemetry.addData("Launch Status", diff <= RPM_TOLERANCE ? "FIRING" : "WAITING FOR SPINUP");
                    }
                }

                if (isAimMode) {
                    telemetry.addData("Err", "%.2f", normalizeAngle(getRobotFieldHeading() - targetHeading));
                    telemetry.addData("Filtered Speed", "%.2f m/s", smoothSpeed);
                }

                telemetry.update();
            }

        } catch (Exception e) {
            telemetry.addData("STATUS", "CRASH PREVENTED");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        } finally {
            if (visionInitialized && vision != null) {
                try { vision.close(); } catch (Exception e) { }
            }
            if (aprilTagLocalizer != null) {
                try { aprilTagLocalizer.close(); } catch (Exception e) { }
            }
            if (intakeMotor != null) intakeMotor.setPower(0);
            if (washer != null) washer.setPower(0);
            if (shMotor != null) shMotor.setPower(0);
            if (mozart != null) mozart.setPower(0);
        }
    }

    private void setLauncherServos(double angle) {
        double clampedAngle = Range.clip(angle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        double fraction = (clampedAngle - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);

        double rpPos = 0.1 + fraction * (0.91 - 0.1);
        double lpPos = 1.0 + fraction * (0.078 - 1.0);

        servoRP.setPosition(rpPos);
        servoLP.setPosition(lpPos);
    }

    private double x(double v) { return Math.pow(v, 3); }
    private double y(double v) { return Math.pow(v, 3); }
    private double rx(double v) { return Math.pow(v, 3); }

    private boolean performVisionCorrection(double currentSpeed) {
        Pose2D visionPose = aprilTagLocalizer.getRobotPose();

        if (visionPose != null) {
            if (currentSpeed < VISION_CORRECT_THRESHOLD) {
                pinpointPoseProvider.setPose(visionPose);
                double visionHeading = visionPose.getHeading(AngleUnit.DEGREES);
                double rawImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                headingOffset = visionHeading - rawImuYaw;
                lastVisionUpdateTime = runtime.seconds();
                return true;
            }
        }
        return false;
    }

    private double getRobotFieldHeading() {
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double correctedYaw = rawYaw + headingOffset;
        return normalizeAngle(correctedYaw);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public class SimpleKalmanFilter {
        private double err_measure;
        private double err_estimate;
        private double q;
        private double current_estimate = 0;
        private double last_estimate = 0;
        private double kalman_gain = 0;

        public SimpleKalmanFilter(double mea_e, double est_e, double q) {
            this.err_measure = mea_e;
            this.err_estimate = est_e;
            this.q = q;
        }

        public double updateEstimate(double mea) {
            kalman_gain = err_estimate / (err_estimate + err_measure);
            current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
            err_estimate = (1.0 - kalman_gain) * err_estimate + Math.abs(last_estimate - current_estimate) * q;
            last_estimate = current_estimate;

            return current_estimate;
        }
    }
}