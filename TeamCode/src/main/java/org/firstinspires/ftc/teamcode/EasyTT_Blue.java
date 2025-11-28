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

@TeleOp(name = "TeleOpBlue", group = "Competition")
public class EasyTT_Blue extends LinearOpMode {

    // --- 硬件定义 ---
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

    // --- 视觉与定位 ---
    private ColorClassifier vision = new ColorClassifier();
    private boolean visionInitialized = false;
    private IMU imu = null;

    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private ArcherLogic archerLogic;
    private ElapsedTime runtime = new ElapsedTime();

    // --- 状态变量 ---
    private boolean isIntakeActive = false;
    private boolean lastLeftBumper = false;

    private boolean isAimMode = false;
    private boolean lastRightBumper = false;

    private double lastDetectedDirection = -1.0;
    private boolean hasDetectedAnyColor = false;
    private boolean isMozartBraked = false;

    private double headingOffset = 0.0;
    private String targetAlliance = "Blue";
    private double lastVisionUpdateTime = 0.0;

    private LaunchSolution currentSolution = null;
    private LaunchSolution lastValidSolution = null;

    private SimpleKalmanFilter angleFilter = new SimpleKalmanFilter(2.0, 1.0, 0.1);
    private double filteredLauncherAngle = 67.5;

    private double smoothSpeed = 0.0;
    private final double SPEED_FILTER_ALPHA = 0.7;

    // --- 常量设置 ---
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

    // 驾驶员修正角度
    private static final double DRIVER_ORIENTATION_CORRECTION = -90.0;

    @Override
    public void runOpMode() {

        // --- 硬件初始化 ---
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
            telemetry.addData("Error", "SH Motor Not Found");
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

                // --- 1. 定位与状态更新 ---
                pinpointPoseProvider.update();

                double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
                double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);
                double normalizationX = robotX_cm / 365.76;
                double normalizationY = robotY_cm / 365.76;

                double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
                double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
                double rawSpeed = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);

                smoothSpeed = (SPEED_FILTER_ALPHA * rawSpeed) + ((1.0 - SPEED_FILTER_ALPHA) * smoothSpeed);
                if (smoothSpeed < 0.1) smoothSpeed = 0.0;

                // --- 2. 视觉定位修正逻辑 ---
                boolean isFiringTrigger = gamepad1.left_trigger > 0.05;

                if (smoothSpeed < VISION_DETECT_THRESHOLD && !isFiringTrigger) {
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
                    if (isFiringTrigger) {
                        visionStatus = "Disabled (Firing)";
                    } else {
                        visionStatus = "Disabled (Too Fast)";
                    }
                }

                // --- 3. 弹道解算 (ArcherLogic) ---
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

                // --- 4. 自瞄模式切换 ---
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

                // --- 5. 飞轮 (Launcher) 控制 ---
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

                // --- 6. 角度舵机 (Servo) 控制 ---
                double rawTargetAngle = (currentSolution != null) ? currentSolution.launcherAngle : 67.5;
                rawTargetAngle = Range.clip(rawTargetAngle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
                filteredLauncherAngle = angleFilter.updateEstimate(rawTargetAngle);

                if (isAimMode) {
                    setLauncherServos(filteredLauncherAngle);
                } else {
                    servoRP.setPosition(0.91);
                    servoLP.setPosition(0.078);
                }

                // --- 7. 底盘运动控制 (Drive) ---
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

                // 底盘矢量控制 + 驾驶员角度修正 (Driver Orientation Correction)
                double botHeadingRad = Math.toRadians(getRobotFieldHeading());
                double drivingHeadingRad = botHeadingRad + Math.toRadians(DRIVER_ORIENTATION_CORRECTION);

                double rotX = x(rawX) * Math.cos(-drivingHeadingRad) - y(rawY) * Math.sin(-drivingHeadingRad);
                double rotY = x(rawX) * Math.sin(-drivingHeadingRad) + y(rawY) * Math.cos(-drivingHeadingRad);

                rotX = rotX * 1.1;
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationPower), 1);
                leftFrontDrive.setPower((rotY + rotX + rotationPower) / denominator);
                leftBehindDrive.setPower((rotY - rotX + rotationPower) / denominator);
                rightFrontDrive.setPower((rotY - rotX - rotationPower) / denominator);
                rightBehindDrive.setPower((rotY + rotX - rotationPower) / denominator);

                // =================================================================
                // 统一机构控制逻辑 (Unified Subsystem Control)
                // =================================================================

                // --- A. 获取状态 ---
                boolean isRpmReady = false;
                if (shMotor != null) {
                    double rpmError = Math.abs(targetRPM - currentRpm);
                    isRpmReady = (rpmError <= RPM_TOLERANCE);
                }

                // 收料开关逻辑
                boolean currentLeftBumper = gamepad1.left_bumper;
                if (currentLeftBumper && !lastLeftBumper) {
                    isIntakeActive = !isIntakeActive;
                    if (isIntakeActive) {
                        isMozartBraked = false; // 重启收料时重置刹车
                        if (visionInitialized) vision.setEnabled(true);
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else {
                        if (visionInitialized) vision.setEnabled(false);
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }
                }
                lastLeftBumper = currentLeftBumper;

                // --- B. 决策树 (优先级：开火 > 收料 > 待机) ---
                double targetMozartPower = 0.0;
                double targetIntakePower = 0.0;
                double targetWasherPower = 0.0;
                double targetHoldPower = 0.0;
                double targetClassifyPower = 0.0;

                if (isFiringTrigger) {
                    // >>> 优先级 1: 开火模式 <<<
                    // 只要扣动扳机，强制覆盖收料逻辑

                    if (isRpmReady) {
                        targetMozartPower = 1.0;
                        targetHoldPower = 1.0;
                        targetClassifyPower = 1.0;
                    } else {
                        // RPM不足，等待
                        targetMozartPower = 0.0;
                        targetHoldPower = 0.0;
                        targetClassifyPower = 0.0;
                    }

                    // 开火时暂停前方收料防止干扰 (可选，若需要边吸边打可改为 1.0)
                    targetIntakePower = 0.0;
                    targetWasherPower = 0.0;

                    // 重置满弹检测，确保能把球打出去
                    isMozartBraked = false;

                } else if (isIntakeActive) {
                    // >>> 优先级 2: 收料模式 <<<
                    targetIntakePower = 1.0;
                    targetWasherPower = 1.0;
                    targetHoldPower = 1.0;

                    // 视觉分类逻辑
                    if (!visionInitialized) {
                        targetClassifyPower = 1.0;
                    } else {
                        ColorClassifier.DetectionResult result = vision.getResult();
                        if (result == ColorClassifier.DetectionResult.GREEN) {
                            targetClassifyPower = -1.0;
                            lastDetectedDirection = 1.0;
                            hasDetectedAnyColor = true;
                        } else if (result == ColorClassifier.DetectionResult.PURPLE) {
                            targetClassifyPower = 1.0;
                            lastDetectedDirection = -1.0;
                            hasDetectedAnyColor = true;
                        } else {
                            if (hasDetectedAnyColor) {
                                targetClassifyPower = -lastDetectedDirection * 0.5;
                            } else {
                                targetClassifyPower = 1.0;
                            }
                        }
                    }

                    // 满弹自动刹车 (仅在 Mozart 未被锁定时检测)
                    if (!isMozartBraked && mozart != null && ballSensor != null) {
                        NormalizedRGBA colors = ballSensor.getNormalizedColors();
                        if (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1) {
                            isMozartBraked = true;
                        }
                    }

                    if (isMozartBraked) {
                        targetMozartPower = 0.0;
                    } else {
                        targetMozartPower = 1.0;
                    }

                } else {
                    // >>> 优先级 3: 待机模式 <<<
                    // 全停
                }

                // --- C. 执行硬件写入 ---
                if (intakeMotor != null) intakeMotor.setPower(targetIntakePower);
                if (washer != null) washer.setPower(targetWasherPower);
                if (holdServo != null) holdServo.setPower(targetHoldPower);
                if (classifyServo != null) classifyServo.setPower(targetClassifyPower);
                if (mozart != null) mozart.setPower(targetMozartPower);

                // =================================================================

                // --- Telemetry ---
                telemetry.addData("Mode", isAimMode ? "AIMING (RUN-N-GUN)" : "Manual");
                telemetry.addData("Vision Status", visionStatus);
                telemetry.addData("Driver Offset", "%.1f deg", DRIVER_ORIENTATION_CORRECTION);

                if (shMotor != null) {
                    telemetry.addData("SH Tgt/Cur RPM", "%.0f / %.0f", targetRPM, currentRpm);
                    if (isAimMode && isFiringTrigger) {
                        telemetry.addData("Launch", isRpmReady ? "FIRING" : "SPINNING UP");
                    }
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