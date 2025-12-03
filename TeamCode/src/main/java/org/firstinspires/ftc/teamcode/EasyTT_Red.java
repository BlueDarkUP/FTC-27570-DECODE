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
    private boolean visionInitialized = false;
    private IMU imu = null;

    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private ArcherLogic archerLogic;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean isIntakeActive = false;

    private boolean lastAButton = false;

    private boolean isAimMode = false;
    private boolean lastRightBumper = false;

    private boolean isManualStopped = false;

    private double lastDetectedDirection = -1.0;
    private boolean hasDetectedAnyColor = false;
    private boolean isMozartBraked = false;

    private double headingOffset = 0.0;
    private final String targetAlliance = "Red";
    private double lastVisionUpdateTime = 0.0;

    private LaunchSolution currentSolution = null;
    private LaunchSolution lastValidSolution = null;

    private SimpleKalmanFilter angleFilter = new SimpleKalmanFilter(2.0, 1.0, 0.1);
    private double filteredLauncherAngle = 67.5;

    // 0.25 表示新值占 25% 权重，旧值占 75%。数值越小越平滑，越大反应越快。
    private static final double AZIMUTH_FILTER_ALPHA = 0.25;
    private double smoothedTargetHeading = 0.0;
    private boolean isFirstAimLoop = true;

    private double smoothSpeed = 0.0;
    private final double SPEED_FILTER_ALPHA = 0.7;

    private static final double VISION_CORRECT_THRESHOLD = 0.05;
    private static final double VISION_DETECT_THRESHOLD = 0.5;

    private static final double MIN_ANGLE_DEG = 39.999999999998;
    private static final double MAX_ANGLE_DEG = 67.444444444448;

    private double targetHeading = 0.0;
    private double headingLastError = 0.0;
    private double lastPidTime = 0.0;

    private static final double P_TURN = 0.0101;
    private static final double D_TURN = 0.0005;
    private static final double K_STATIC = 0.12;
    private static final double MAX_AUTO_TURN = 1;

    private double manualTargetRPM = 1500.0;
    private double manualTargetAngle = 67.44;
    private double currentRpmTolerance = 50.0;

    private double lastRumbleTime = 0.0;
    private String visionStatus = "Ready";

    private boolean lastRpmReady = false;

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
        setLauncherServos(manualTargetAngle);

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

        telemetry.addLine("Ready. Alliance Locked to RED.");
        telemetry.addLine("Controls: A=Intake, X=Stop, Y=Fire, RB=Aim, R-Stick=Turn");
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

                aprilTagLocalizer.updateDecimationByNormalizedPos(normalizationX, normalizationY);

                double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
                double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
                double rawSpeed = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);

                smoothSpeed = (SPEED_FILTER_ALPHA * rawSpeed) + ((1.0 - SPEED_FILTER_ALPHA) * smoothSpeed);
                if (smoothSpeed < 0.1) smoothSpeed = 0.0;

                boolean isFiringTrigger = gamepad1.y;

                if (isFiringTrigger) {
                    isManualStopped = false;
                }

                if (smoothSpeed < VISION_DETECT_THRESHOLD) {
                    boolean corrected = performVisionCorrection(smoothSpeed);
                    if (corrected) {
                        visionStatus = "Correcting (Active)";
                        if (!isFiringTrigger && runtime.seconds() - lastRumbleTime > 1.0) {
                            gamepad1.rumbleBlips(2);
                            lastRumbleTime = runtime.seconds();
                        }
                    } else {
                        visionStatus = "Monitoring (No Replace)";
                    }
                } else {
                    visionStatus = "Disabled (Too Fast/Stop)";
                }

                double direction_deg = Math.toDegrees(Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s));
                if (direction_deg < 0) direction_deg += 360;

                if (gamepad1.x) {
                    isManualStopped = true;
                    isIntakeActive = false;
                    isAimMode = false;
                    manualTargetRPM = 1500.0;
                    isMozartBraked = false;
                    if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                    gamepad1.rumbleBlips(1);
                }

                if (gamepad1.dpad_right) {
                    isManualStopped = false;
                    manualTargetRPM = 3280;
                    currentRpmTolerance = 100;
                    manualTargetAngle = calculateAngleFromPos(0.0);
                }
                if (gamepad1.dpad_left) {
                    isManualStopped = false;
                    manualTargetRPM = 2400;
                    currentRpmTolerance = 150;
                    manualTargetAngle = calculateAngleFromPos(0.7);
                }
                if (gamepad1.dpad_down) {
                    isManualStopped = false;
                    manualTargetRPM = 2000;
                    currentRpmTolerance = 150;
                    manualTargetAngle = calculateAngleFromPos(1.0);
                }
                if (gamepad1.dpad_up) {
                    isManualStopped = false;
                    manualTargetRPM = 2800;
                    currentRpmTolerance = 100;
                    manualTargetAngle = calculateAngleFromPos(0.2);
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
                    if (isManualStopped) isManualStopped = false;

                    isAimMode = !isAimMode;
                    if (isAimMode) {
                        headingLastError = 0;
                        lastPidTime = runtime.seconds();

                        isFirstAimLoop = true;

                        if (currentSolution == null) {
                            targetHeading = getRobotFieldHeading();
                            smoothedTargetHeading = targetHeading;
                        }
                    }
                }
                lastRightBumper = currentRightBumper;

                double activeTargetRPM = manualTargetRPM;
                double activeTargetAngle = manualTargetAngle;

                if (isManualStopped && !isFiringTrigger) {
                    activeTargetRPM = 1500.0;
                }

                if (isAimMode && currentSolution != null && !isManualStopped) {
                    double rawHeadingDeg = currentSolution.aimAzimuthDeg + 180;

                    if (isFirstAimLoop) {
                        smoothedTargetHeading = rawHeadingDeg;
                        isFirstAimLoop = false;
                    } else {
                        smoothedTargetHeading = (AZIMUTH_FILTER_ALPHA * rawHeadingDeg) +
                                ((1.0 - AZIMUTH_FILTER_ALPHA) * smoothedTargetHeading);
                    }

                    targetHeading = normalizeAngle(smoothedTargetHeading);

                    activeTargetRPM = currentSolution.motorRpm;
                    activeTargetAngle = currentSolution.launcherAngle;
                    currentRpmTolerance = 50.0;
                }

                if (shMotor != null) {
                    double targetVelocityTPS = (activeTargetRPM * MOTOR_TICK_COUNT) / 60.0;
                    shMotor.setVelocity(targetVelocityTPS);
                }

                activeTargetAngle = Range.clip(activeTargetAngle, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
                filteredLauncherAngle = angleFilter.updateEstimate(activeTargetAngle);
                setLauncherServos(filteredLauncherAngle);

                if (gamepad1.right_stick_button) {
                    imu.resetYaw();
                    headingOffset = -DRIVER_ORIENTATION_CORRECTION;
                }

                double rawY = -gamepad1.left_stick_y;
                double rawX = gamepad1.left_stick_x;
                double rotationPower = 0.0;

                if (isAimMode && !isManualStopped) {
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
                    double rightStickTurn = gamepad1.right_stick_x;
                    rotationPower = rx(rightStickTurn);
                }

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

                boolean isRpmReady = false;
                double currentRpmForCheck = 0.0;
                if (shMotor != null) {
                    double currentVel = shMotor.getVelocity();
                    currentRpmForCheck = (currentVel * 60.0) / MOTOR_TICK_COUNT;
                    double rpmError = Math.abs(activeTargetRPM - currentRpmForCheck);
                    isRpmReady = (rpmError <= currentRpmTolerance);

                    if (isFiringTrigger && lastRpmReady && !isRpmReady) {
                        gamepad1.rumble(1.0, 1.0, 150);
                    }
                    lastRpmReady = isRpmReady;
                }

                boolean currentAButton = gamepad1.a;
                if (currentAButton && !lastAButton) {
                    if (isManualStopped) isManualStopped = false;

                    isIntakeActive = !isIntakeActive;
                    if (isIntakeActive) {
                        isMozartBraked = false;
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else {
                        if (mozart != null) mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    }
                }
                lastAButton = currentAButton;

                double targetMozartPower = 0.0;
                double targetIntakePower = 0.0;
                double targetWasherPower = 0.0;
                double targetHoldPower = 0.0;
                double targetClassifyPower = 0.0;

                if (isFiringTrigger) {
                    if (isRpmReady) {
                        targetMozartPower = 1.0;
                        targetHoldPower = 1.0;
                        targetClassifyPower = 1.0;
                    } else {
                        targetMozartPower = 0.0;
                        targetHoldPower = 0.0;
                        targetClassifyPower = 0.0;
                    }

                    targetIntakePower = 0.0;
                    targetWasherPower = 0.0;
                    isMozartBraked = false;

                } else if (isIntakeActive && !isManualStopped) {
                    targetIntakePower = 1.0;
                    targetWasherPower = 1.0;
                    targetHoldPower = 1.0;

                    if (!visionInitialized) {
                        targetClassifyPower = 1.0;
                    }

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
                }

                if (isManualStopped && !isFiringTrigger) {
                    targetIntakePower = 0;
                    targetWasherPower = 0;
                    targetHoldPower = 0;
                    targetClassifyPower = 0;
                    targetMozartPower = 0;
                }

                if (intakeMotor != null) intakeMotor.setPower(targetIntakePower);
                if (washer != null) washer.setPower(targetWasherPower);
                if (holdServo != null) holdServo.setPower(targetHoldPower);
                if (classifyServo != null) classifyServo.setPower(targetClassifyPower);
                if (mozart != null) mozart.setPower(targetMozartPower);

                if (isManualStopped) {
                    telemetry.addData("WARNING", "EMERGENCY STOP (Press A/RB/Y to Reset)");
                }
                telemetry.addData("Mode", isAimMode ? "AIMING (Auto)" : "MANUAL (D-Pad)");
                telemetry.addData("Alliance", targetAlliance);
                telemetry.addData("Target RPM", "%.0f (Tol: %.0f)", activeTargetRPM, currentRpmTolerance);
                telemetry.addData("Target Angle", "%.2f deg", activeTargetAngle);

                if (shMotor != null) {
                    telemetry.addData("Launch", isFiringTrigger ? (isRpmReady ? "FIRING" : "WAIT RPM") : "IDLE");
                    telemetry.addData("Cur RPM", "%.0f", currentRpmForCheck);
                }

                telemetry.update();
            }

        } catch (Exception e) {
            telemetry.addData("STATUS", "CRASH PREVENTED");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        } finally {
            if (aprilTagLocalizer != null) {
                try { aprilTagLocalizer.close(); } catch (Exception e) { }
            }
            if (intakeMotor != null) intakeMotor.setPower(0);
            if (washer != null) washer.setPower(0);
            if (shMotor != null) shMotor.setPower(0);
            if (mozart != null) mozart.setPower(0);
        }
    }

    private double calculateAngleFromPos(double pos) {
        double clampedPos = Range.clip(pos, 0.0, 1.0);
        return MIN_ANGLE_DEG + clampedPos * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
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