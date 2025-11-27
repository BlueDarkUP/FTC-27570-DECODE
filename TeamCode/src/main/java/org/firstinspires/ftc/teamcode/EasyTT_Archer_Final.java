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

@TeleOp(name = "EasyTT_Archer_Final_Corrected", group = "Competition")
public class EasyTT_Archer_Final extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBehindDrive = null;
    private DcMotor rightBehindDrive = null;

    private DcMotor intakeMotor = null;

    // SH Motor 定义
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

    private SimpleKalmanFilter angleFilter = new SimpleKalmanFilter(2.0, 1.0, 0.1);
    private double filteredLauncherAngle = 67.5;

    private static final double MIN_ANGLE_DEG = 39.999999999998;
    private static final double MAX_ANGLE_DEG = 67.444444444448;

    private double targetHeading = 0.0;
    private double headingLastError = 0.0;
    private double lastPidTime = 0.0;

    private static final double P_TURN = 0.01;
    private static final double D_TURN = 0.00;
    private static final double K_STATIC = 0.0;
    private static final double MAX_AUTO_TURN = 1;

    // 新增：转速允许误差范围 (RPM)
    private static final double RPM_TOLERANCE = 80.0;

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

        // --- SH Motor 初始化 ---
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
            performVisionCorrection();

            while (opModeIsActive()) {

                // 1. 更新定位
                boolean visionUpdated = performVisionCorrection();
                pinpointPoseProvider.update();

                // 2. 获取机器人状态
                double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
                double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);
                double normalizationX = robotX_cm / 365.76;
                double normalizationY = robotY_cm / 365.76;

                double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
                double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
                double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);
                double direction_deg = Math.toDegrees(Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s));
                if (direction_deg < 0) direction_deg += 360;

                // 3. 处理按键和解算
                if (gamepad1.x) targetAlliance = "Blue";
                if (gamepad1.b) targetAlliance = "Red";
                if (gamepad1.a) {
                    pinpointPoseProvider.reset();
                    imu.resetYaw();
                    headingOffset = 0;
                }

                CalculationParams currentParams = new CalculationParams(
                        normalizationX, normalizationY, speed_m_s, direction_deg, targetAlliance
                );
                currentSolution = archerLogic.calculateSolution(currentParams);

                boolean currentRightBumper = gamepad1.right_bumper;
                if (currentRightBumper && !lastRightBumper) {
                    isAimMode = !isAimMode;
                    if (isAimMode) {
                        if (currentSolution != null) {
                            targetHeading = normalizeAngle(currentSolution.aimAzimuthDeg + 180);
                        } else {
                            targetHeading = getRobotFieldHeading();
                        }
                        headingLastError = 0;
                        lastPidTime = runtime.seconds();
                    }
                }
                lastRightBumper = currentRightBumper;

                // =============================================================
                // SH Motor 闭环控制逻辑 (SH Motor Closed Loop Control)
                // =============================================================

                double targetRPM = 1000.0; // 默认空闲转速
                double currentRpm = 0.0;

                // 确定目标转速
                if (isAimMode && currentSolution != null) {
                    targetRPM = currentSolution.motorRpm;
                }

                if (shMotor != null) {
                    // 读取当前转速（用于判断发射条件）
                    double currentVel = shMotor.getVelocity();
                    currentRpm = (currentVel * 60.0) / MOTOR_TICK_COUNT;

                    // 设置目标速度
                    double targetVelocityTPS = (targetRPM * MOTOR_TICK_COUNT) / 60.0;
                    shMotor.setVelocity(targetVelocityTPS);
                }

                // =============================================================
                // 舵机与发射逻辑 (Servo & Launch Logic)
                // =============================================================

                double rawTargetAngle = (currentSolution != null) ? currentSolution.launcherAngle : 67.5;
                rawTargetAngle = Range.clip(rawTargetAngle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
                filteredLauncherAngle = angleFilter.updateEstimate(rawTargetAngle);

                if (isAimMode) {
                    // 自瞄模式下控制仰角
                    setLauncherServos(filteredLauncherAngle);

                    // --- [优化的 MOZART 控制] ---
                    if (mozart != null) {
                        if (gamepad1.left_trigger > 0.05) {
                            // 只有当按下扳机 且 实际转速在误差范围内时才发射
                            double rpmError = Math.abs(targetRPM - currentRpm);

                            if (rpmError <= RPM_TOLERANCE) {
                                mozart.setPower(1.0); // 允许发射
                            } else {
                                mozart.setPower(0.0); // 强制等待加速完成 (BRAKE效果)
                            }
                        } else {
                            mozart.setPower(0.0); // 松开扳机停止
                        }
                    }
                } else {
                    // 手动模式归位
                    servoRP.setPosition(0.91);
                    servoLP.setPosition(0.078);
                }

                // =============================================================
                // 底盘控制 (Chassis Control)
                // =============================================================
                if (gamepad1.options) {
                    imu.resetYaw();
                    headingOffset = 0;
                }

                double rawY = -gamepad1.left_stick_y;
                double rawX = gamepad1.left_stick_x;
                double rotationPower;

                if (isAimMode) {
                    // PID 自动对准逻辑...
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

                double botHeading = Math.toRadians(getRobotFieldHeading());
                double rotX = x(rawX) * Math.cos(-botHeading) - y(rawY) * Math.sin(-botHeading);
                double rotY = x(rawX) * Math.sin(-botHeading) + y(rawY) * Math.cos(-botHeading);

                rotX = rotX * 1.1;
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationPower), 1);
                leftFrontDrive.setPower((rotY + rotX + rotationPower) / denominator);
                leftBehindDrive.setPower((rotY - rotX + rotationPower) / denominator);
                rightFrontDrive.setPower((rotY - rotX - rotationPower) / denominator);
                rightBehindDrive.setPower((rotY + rotX - rotationPower) / denominator);

                // =============================================================
                // 进料 (Intake) 控制
                // 注意：如果处于 AimMode，Intake 逻辑不应干扰 MOZART
                // =============================================================

                boolean currentLeftBumper = gamepad1.left_bumper;
                if (currentLeftBumper && !lastLeftBumper) {
                    isIntakeActive = !isIntakeActive;
                    if (isIntakeActive) {
                        isMozartBraked = false;
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else {
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }
                }
                lastLeftBumper = currentLeftBumper;

                if (isIntakeActive) {
                    intakeMotor.setPower(1.0);
                    washer.setPower(1.0);
                    holdServo.setPower(-1.0);

                    // 视觉与传感器逻辑
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

                    // --- [修正 Intake 对 MOZART 的控制] ---
                    // 只有在非自瞄模式下，Intake 逻辑才能控制 MOZART
                    // 如果在自瞄，MOZART 由上面的 Trigger 逻辑全权负责
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

                    // 同样，只有非自瞄模式下才强制关闭
                    if (!isAimMode && mozart != null) mozart.setPower(0.0);
                }

                // =============================================================
                // Telemetry
                // =============================================================
                telemetry.addData("Mode", isAimMode ? "AIMING (BACK)" : "Manual");

                if (shMotor != null) {
                    telemetry.addData("SH Target RPM", "%.1f", targetRPM);
                    telemetry.addData("SH Current RPM", "%.1f", currentRpm);
                    if (isAimMode && gamepad1.left_trigger > 0.05) {
                        double diff = Math.abs(targetRPM - currentRpm);
                        telemetry.addData("Launch Status", diff <= RPM_TOLERANCE ? "FIRING" : "WAITING FOR SPINUP");
                    }
                }

                if (isAimMode) {
                    telemetry.addData("Err", "%.2f", normalizeAngle(getRobotFieldHeading() - targetHeading));
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

    private boolean performVisionCorrection() {
        Pose2D visionPose = aprilTagLocalizer.getRobotPose();
        if (visionPose != null) {
            pinpointPoseProvider.setPose(visionPose);
            double visionHeading = visionPose.getHeading(AngleUnit.DEGREES);
            double rawImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            headingOffset = visionHeading - rawImuYaw;
            lastVisionUpdateTime = runtime.seconds();
            return true;
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