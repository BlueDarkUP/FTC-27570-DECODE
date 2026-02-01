package org.firstinspires.ftc.teamcode.auto.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "红方远点", group = "Main")
public class RedFarAutoCycle extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState = 0;
    private int loopCount = 0;

    // --- 配置常量 ---
    private final double FarShootingRPM = 3400.0;
    private final double IdleRPM = 1500.0;
    private final double RPM_TOLERANCE = 300.0;

    // 平滑降速逻辑
    private double currentSlewRPM = 0;
    private final double RPM_DOWN_STEP = 8.0;

    // --- 硬件定义 ---
    private DcMotorEx SH, HS, Intake, Mozart;
    private CRServo Hold;
    private Servo servoRP = null, servoLP = null;
    private DigitalChannel juju;
    private Servo lightLeft, lightRight;

    private enum LightState { IDLE, TRANSITIONING, ANIMATING }
    private RedFarAutoCycle.LightState currentLightState = RedFarAutoCycle.LightState.IDLE;
    private ElapsedTime lightTimer = new ElapsedTime();

    private double animationMin = 0.0, animationMax = 0.0, animationDurationSec = 1.0;
    private boolean isFadingUp = true;

    private double transitionFromPos = 0.0, transitionToPos = 0.0, targetMin = 0.0, targetMax = 0.0;
    private static final double TRANSITION_DURATION_SEC = 0.5;

    private static final double C_OFF = 0.1;
    private static final double C_RED = 0.279;
    private static final double C_ORANGE = 0.333;
    private static final double C_YELLOW = 0.388;
    private static final double C_SAGE = 0.444;
    private static final double C_GREEN = 0.500;
    private static final double C_Azure = 0.555;
    private static final double C_BLUE = 0.611;
    private static final double C_Indigo = 0.666;
    private static final double C_VIOLET = 0.722;
    private static final double C_WHITE = 0.8;


    // --- 飞轮 PID 变量 ---
    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double targetShooterRPM = 0;
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.006, SHOOTER_F = 0.0004, SHOOTER_D = 0.00001, TICKS_PER_REV = 28.0;

    // --- 机构控制变量 ---
    private boolean isIntakeActive = false;
    private boolean hasCaughtObject = false;
    private ElapsedTime shootActionTimer = new ElapsedTime();
    private double shootTimeLimitSec = 0;
    private boolean isShootingTaskActive = false;
    private boolean isChassisPausedByShot = false;

    // --- 路径定义 ---
    private final Pose startPose = new Pose(88.000, 8.000, Math.toRadians(-90));
    private PathChain Path1, Path2, Path3, Path4, Path5;

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        shooterPidTimer.reset();

        SH = hardwareMap.get(DcMotorEx.class, "SH");
        HS = hardwareMap.get(DcMotorEx.class, "HS");
        SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        HS.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Mozart = hardwareMap.get(DcMotorEx.class, "MOZART");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        juju = hardwareMap.get(DigitalChannel.class, "bigjuju");
        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");

        lightLeft = hardwareMap.get(Servo.class, "lightLeft");
        lightRight = hardwareMap.get(Servo.class, "lightRight");

        juju.setMode(DigitalChannel.Mode.INPUT);
        Mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        servoLP.setPosition(0.925);
        servoRP.setPosition(0.03);
        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
        // 初始状态：蓝色呼吸灯 (Blue Alliance)
        setLightAnimation(C_BLUE, C_Indigo, 1.5);
    }

    @Override
    public void loop() {
        updateShooterPID();
        // 发射逻辑现在内部包含了转速检测
        if (isShootingTaskActive) runShootingSequence(); else updateIntakeLogic();

        follower.update();
        autonomousPathUpdate();
        updateLights();
        telemetry.addData("State", pathState);
        telemetry.addData("Current RPM", "%.0f", getShooterRPM());
        telemetry.addData("Mozart Status", isShootingTaskActive ? (Math.abs(getShooterRPM() - targetShooterRPM) <= RPM_TOLERANCE ? "FIRING" : "WAITING") : "IDLE");
        telemetry.update();
    }
    public void setLightAnimation(double min, double max, double durationSec) {
        if (targetMin == min && targetMax == max && Math.abs(animationDurationSec - durationSec) < 0.01) {
            return;
        }

        this.targetMin = min;
        this.targetMax = max;
        this.animationDurationSec = durationSec;

        this.transitionFromPos = lightLeft.getPosition();
        this.transitionToPos = this.targetMin;

        this.currentLightState = RedFarAutoCycle.LightState.TRANSITIONING;
        this.lightTimer.reset();
    }

    public void updateLights() {
        double currentPosition = lightLeft.getPosition();
        double elapsedTime = lightTimer.seconds();

        switch (currentLightState) {
            case IDLE:
                break;

            case TRANSITIONING:
                double transitionProgress = Math.min(elapsedTime / TRANSITION_DURATION_SEC, 1.0);
                currentPosition = transitionFromPos + (transitionToPos - transitionFromPos) * transitionProgress;

                if (transitionProgress >= 1.0) {
                    currentPosition = transitionToPos;
                    animationMin = targetMin;
                    animationMax = targetMax;
                    isFadingUp = true;
                    currentLightState = RedFarAutoCycle.LightState.ANIMATING;
                    lightTimer.reset();
                }
                break;

            case ANIMATING:
                double animationProgress = Math.min(elapsedTime / animationDurationSec, 1.0);

                if (isFadingUp) {
                    currentPosition = animationMin + (animationMax - animationMin) * animationProgress;
                } else {
                    currentPosition = animationMax - (animationMax - animationMin) * animationProgress;
                }

                if (animationProgress >= 1.0) {
                    isFadingUp = !isFadingUp;
                    lightTimer.reset();
                }
                break;
        }

        lightLeft.setPosition(currentPosition);
        lightRight.setPosition(currentPosition);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setLightAnimation(C_BLUE, C_Indigo, 1.5);
                targetShooterRPM = FarShootingRPM;
                follower.followPath(Path1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        setLightAnimation(C_RED,C_ORANGE,0.5);
                        shooting(1000, false);
                        setPathState(2);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 2:
                if (!isShootingTaskActive) {
                    setLightAnimation(C_ORANGE,C_SAGE,0.5);
                    targetShooterRPM = IdleRPM;
                    loopCount = 0;
                    setPathState(10);
                }
                break;
            case 10:
                if (loopCount < 3) {
                    follower.followPath(Path2, false);
                    setPathState(11);
                } else {
                    setPathState(20);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setLightAnimation(C_Azure,C_VIOLET,2);
                    setIntake(true);
                    follower.setMaxPower(0.7);
                    follower.followPath(Path3, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                        setLightAnimation(C_BLUE, C_Indigo, 1.5);
                        targetShooterRPM = FarShootingRPM;
                        follower.setMaxPower(1);
                        follower.followPath(Path4, true);
                        setPathState(13);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        setLightAnimation(C_RED,C_ORANGE,0.5);
                        shooting(1000, false);
                        setPathState(14);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 14:
                if (!isShootingTaskActive) {
                    setLightAnimation(C_ORANGE,C_SAGE,0.5);
                    targetShooterRPM = IdleRPM;
                    setIntake(false);
                    loopCount++;
                    setPathState(10);
                }
                break;
            case 20:
                targetShooterRPM = 0;
                setLightAnimation(C_RED,C_VIOLET,0.8);
                follower.followPath(Path5, true);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    // ================= 核心：带门控的发射逻辑 =================

    private void runShootingSequence() {
        if (shootActionTimer.seconds() < shootTimeLimitSec) {
            // Intake 和 Hold 始终强制开启
            Intake.setPower(1.0);
            Hold.setPower(1.0);

            // --- 理想射击范围门控 ---
            double currentRPM = getShooterRPM();
            // 只有当转速处于 目标值 ± 70 RPM 之间时，才驱动 Mozart
            if (Math.abs(currentRPM - targetShooterRPM) <= RPM_TOLERANCE) {
                Mozart.setPower(0.8); // 正常供弹
            } else {
                Mozart.setPower(0);   // 掉速严重，立即刹车等待恢复
            }
        } else {
            // 任务结束，彻底停止
            isShootingTaskActive = false;
            Intake.setPower(0);
            Mozart.setPower(0);
            Hold.setPower(0);
            if (isChassisPausedByShot) follower.resumePathFollowing();
        }
    }

    // ================= 系统功能函数 (保持不变) =================

    public void shooting(long millis, boolean holdChassis) {
        shootTimeLimitSec = millis / 1000.0;
        isShootingTaskActive = true;
        isChassisPausedByShot = holdChassis;
        shootActionTimer.reset();
        if (holdChassis) follower.pausePathFollowing();
    }

    public void setIntake(boolean active) {
        if (active && !isIntakeActive) hasCaughtObject = false;
        isIntakeActive = active;
    }

    private void updateIntakeLogic() {
        if (isIntakeActive) {
            Intake.setPower(1.0); Hold.setPower(1.0);
            if (!hasCaughtObject && juju.getState()) hasCaughtObject = true;
            Mozart.setPower(hasCaughtObject ? 0 : 0.6);
        } else {
            Intake.setPower(0); Hold.setPower(0); Mozart.setPower(0);
        }
    }

    private void updateShooterPID() {
        if (currentSlewRPM > targetShooterRPM) {
            currentSlewRPM -= RPM_DOWN_STEP;
            if (currentSlewRPM < targetShooterRPM) currentSlewRPM = targetShooterRPM;
        } else {
            currentSlewRPM = targetShooterRPM;
        }
        double currentVelTPS = SH.getVelocity();
        double targetVelTPS = (currentSlewRPM * TICKS_PER_REV) / 60.0;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt == 0) dt = 1e-9;
        double errorTPS = targetVelTPS - currentVelTPS;
        double derivative = (errorTPS - shooterLastError) / dt;
        shooterLastError = errorTPS;
        double finalPower = (SHOOTER_F * targetVelTPS) + (SHOOTER_P * errorTPS) + (SHOOTER_D * derivative);
        if (targetShooterRPM == 0 && currentSlewRPM == 0) finalPower = 0;
        SH.setPower(finalPower); HS.setPower(finalPower);
    }

    private double getShooterRPM() { return (SH.getVelocity() * 60.0) / TICKS_PER_REV; }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(88.500, 14.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.500, 14.000), new Pose(132.350, 22.000)))
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(300))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.350, 22.000), new Pose(136.600, 8.400)))
                .setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(270))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(136.600, 8.400), new Pose(80.000, 75), new Pose(81.500, 14.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(81.500, 14.000), new Pose(120.300, 75.000), new Pose(130, 8.490)))
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}