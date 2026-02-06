package org.firstinspires.ftc.teamcode.auto.Auto2Gate;

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
import org.firstinspires.ftc.teamcode.drive.LedModuleDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "蓝方开两次门", group = "Main")
public class Blue2Gate extends OpMode {

    private Follower follower;
    private Timer pathTimer, cycleTimer, actionTimer;
    private int pathState = 0;

    // --- 射击参数 ---
    private final double FirstShootingPower = 2800.0;
    private final double NormalShootingPower = 2600.0;
    private final double IdleShootingPower = 1500.0;

    // --- 硬件定义 ---
    private DcMotorEx SH, HS, Intake, Mozart;
    private CRServo Hold;
    private Servo servoRP = null;
    private Servo servoLP = null;
    private DigitalChannel juju;
    private Servo lightLeft, lightRight;
    LedModuleDriver led;

    // --- 灯光变量 ---
    private enum LightState { IDLE, TRANSITIONING, ANIMATING }
    private LightState currentLightState = LightState.IDLE;
    private ElapsedTime lightTimer = new ElapsedTime();
    final char ID = 'A';
    private String lastLedState = "";
    private double animationMin = 0.0, animationMax = 0.0, animationDurationSec = 1.0;
    private boolean isFadingUp = true;
    private double transitionFromPos = 0.0, transitionToPos = 0.0, targetMin = 0.0, targetMax = 0.0;
    private static final double TRANSITION_DURATION_SEC = 0.5;

    // --- 灯光颜色常量 ---
    private static final double C_RED = 0.279;
    private static final double C_ORANGE = 0.333;
    private static final double C_SAGE = 0.444;
    private static final double C_GREEN = 0.500;
    private static final double C_Azure = 0.555;
    private static final double C_BLUE = 0.611;
    private static final double C_Indigo = 0.666;
    private static final double C_VIOLET = 0.722;

    // --- 射击PID变量 ---
    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double targetShooterRPM = 0;
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.007, SHOOTER_F = 0.0003, SHOOTER_D = 0.000001, TICKS_PER_REV = 28.0;

    // --- 逻辑标志位 ---
    private boolean isIntakeActive = false;
    private boolean hasCaughtObject = false;
    private ElapsedTime shootActionTimer = new ElapsedTime();
    private double shootTimeLimitSec = 0;
    private boolean isShootingTaskActive = false;
    private boolean isChassisPausedByShot = false;

    // --- 路径变量 ---
    private final Pose startPose = new Pose(27.495, 132.350, Math.toRadians(-36));
    // Path15, Path16 已删除
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9,
            Path10, Path11, Path12, Path13, Path14, Path17;

    @Override
    public void init() {
        pathTimer = new Timer();
        cycleTimer = new Timer();
        actionTimer = new Timer();
        shooterPidTimer.reset();

        // 硬件初始化
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

        led = hardwareMap.get(LedModuleDriver.class, "leddd");
        led.assignId(ID);
        led.setWaveMode('A', 0xFF00FF, 0x00FFFF, 200, 60);

        juju.setMode(DigitalChannel.Mode.INPUT);
        Mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        servoLP.setPosition(0.0);
        servoRP.setPosition(0.925);

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
        setLightAnimation(C_BLUE, C_Indigo, 1.5);
    }

    @Override
    public void loop() {
        updateShooterPID();

        if (isShootingTaskActive) {
            runShootingSequence();
        } else {
            updateIntakeLogic();
        }

        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        updateLights();
        updateSmartLed();

        telemetry.addData("State", pathState);
        telemetry.addData("RPM", getShooterRPM());
        telemetry.update();
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            // ==========================================
            // 阶段 0: 预载射击
            // ==========================================
            case 0:
                targetShooterRPM = FirstShootingPower;
                follower.followPath(Path1, false);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.45) {
                    setLightAnimation(C_RED, C_ORANGE, 0.5);
                    shooting(500, false);
                    servoLP.setPosition(0.30);
                    servoRP.setPosition(0.625);
                    hasCaughtObject = false;
                    setPathState(2);
                }
                break;

            case 2:
                if (!isShootingTaskActive) {
                    targetShooterRPM = IdleShootingPower;
                    setPathState(100);
                }
                break;

            // ==========================================
            // 阶段 1: 第一排样本 (正常开门)
            // ==========================================
            case 100:
                setLightAnimation(C_SAGE, C_GREEN, 1);
                follower.followPath(Path2, false);
                setPathState(101);
                break;

            case 101:
                if (!follower.isBusy()) {
                    setLightAnimation(C_Azure, C_VIOLET, 2);
                    setIntake(true);
                    servoLP.setPosition(0.40);
                    servoRP.setPosition(0.525);
                    follower.setMaxPower(0.9);
                    follower.followPath(Path3, false);
                    setPathState(102);
                }
                break;

            case 102: // 准备开门
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.setMaxPower(1.0);
                    follower.followPath(Path4, false);
                    setPathState(103);
                }
                break;

            case 103: // 开门动作
                if (!follower.isBusy()) {
                    follower.followPath(Path5, false);
                    setPathState(104);
                }
                break;

            case 104: // 前往射击点
                if (!follower.isBusy()) {
                    Thread.sleep(350);
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.followPath(Path6, true);
                    setPathState(105);
                }
                break;

            case 105:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.28) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(106);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 106:
                if (!isShootingTaskActive) {
                    setPathState(200);
                }
                break;

            // ==========================================
            // 阶段 2: 第二排样本 (正常开门)
            // ==========================================
            case 200:
                targetShooterRPM = IdleShootingPower;
                setLightAnimation(C_SAGE, C_GREEN, 1);
                follower.followPath(Path7, false);
                setPathState(201);
                break;

            case 201:
                if (!follower.isBusy()) {
                    setLightAnimation(C_Azure, C_VIOLET, 2);
                    setIntake(true);
                    servoLP.setPosition(0.40);
                    servoRP.setPosition(0.525);
                    follower.setMaxPower(0.9);
                    follower.followPath(Path8, false);
                    setPathState(202);
                }
                break;

            case 202: // 准备开门 (曲线)
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.setMaxPower(1.0);
                    follower.followPath(Path9, false);
                    setPathState(203);
                }
                break;

            case 203: // 开门动作
                if (!follower.isBusy()) {
                    follower.followPath(Path10, false);
                    setPathState(204);
                }
                break;

            case 204: // 前往射击点
                if (!follower.isBusy()) {
                    Thread.sleep(350);
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.followPath(Path11, true);
                    setPathState(205);
                }
                break;

            case 205:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.28) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(206);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 206:
                if (!isShootingTaskActive) {
                    setPathState(300);
                }
                break;

            // ==========================================
            // 阶段 3: 第三排样本 (修改：吸取 -> 直接射击，不开门)
            // ==========================================
            case 300: // 准备第三排
                targetShooterRPM = IdleShootingPower;
                setLightAnimation(C_SAGE, C_GREEN, 1);
                follower.followPath(Path12, false);
                setPathState(301);
                break;

            case 301: // 吸取第三排
                if (!follower.isBusy()) {
                    setLightAnimation(C_Azure, C_VIOLET, 2);
                    setIntake(true);
                    servoLP.setPosition(0.40);
                    servoRP.setPosition(0.525);
                    follower.setMaxPower(0.9);
                    follower.followPath(Path13, false);
                    setPathState(302);
                }
                break;

            case 302: // 【修改点】吸取完毕后，直接前往射击点
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.setMaxPower(1.0);

                    Thread.sleep(100);
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);

                    // 执行修改后的 Path14 (Intake Row 3 -> Shoot Pos)
                    follower.followPath(Path14, true);

                    // 跳过原有的开门 Case 303 和 304，直接射击
                    setPathState(305);
                }
                break;

            // Case 303 (原开门) 已被跳过
            // Case 304 (原门到射击) 已被跳过

            case 305: // 射击第三排样本
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.28) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(306);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            // ==========================================
            // 阶段 4: 停靠
            // ==========================================
            case 306:
                if (!isShootingTaskActive) {
                    setIntake(false);
                    targetShooterRPM = 0;
                    setLightAnimation(C_RED, C_VIOLET, 0.8);
                    follower.followPath(Path17, true);
                    setPathState(400);
                }
                break;

            case 400: // 结束
                led.turnOff(ID);
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    // --- 辅助方法 ---
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    public void shooting(long millis, boolean holdChassis) {
        shootTimeLimitSec = millis / 1000.0;
        isShootingTaskActive = true;
        isChassisPausedByShot = holdChassis;
        shootActionTimer.reset();
        if (holdChassis) follower.pausePathFollowing();
    }

    private void runShootingSequence() {
        if (shootActionTimer.seconds() < shootTimeLimitSec) {
            Intake.setPower(1.0); Mozart.setPower(0.85); Hold.setPower(-1.0);
        } else {
            isShootingTaskActive = false;
            Intake.setPower(0); Mozart.setPower(0); Hold.setPower(0);
            if (isChassisPausedByShot) follower.resumePathFollowing();
        }
    }

    public void setIntake(boolean active) {
        if (active && !isIntakeActive) hasCaughtObject = false;
        isIntakeActive = active;
        if (active) {
            hasCaughtObject = false;
        }
    }

    private void updateIntakeLogic() {
        if (isIntakeActive) {
            Intake.setPower(1.0); Hold.setPower(0);
            if (!hasCaughtObject && juju.getState()) hasCaughtObject = true;
            Mozart.setPower(hasCaughtObject ? 0 : 0.6);
        } else {
            Intake.setPower(0); Hold.setPower(0); Mozart.setPower(0);
        }
    }

    private void updateShooterPID() {
        double currentVelTPS = SH.getVelocity();
        double targetVelTPS = (targetShooterRPM * TICKS_PER_REV) / 60.0;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt == 0) dt = 1e-9;
        double errorTPS = targetVelTPS - currentVelTPS;
        double derivative = (errorTPS - shooterLastError) / dt;
        shooterLastError = errorTPS;
        double finalPower = (SHOOTER_F * targetVelTPS) + (SHOOTER_P * errorTPS) + (SHOOTER_D * derivative);
        if (targetShooterRPM == 0) finalPower = 0;
        SH.setPower(finalPower); HS.setPower(finalPower);
    }

    private double getShooterRPM() { return (SH.getVelocity() * 60.0) / TICKS_PER_REV; }

    public void updateSmartLed() {
        String reqState = "";
        if (isShootingTaskActive) reqState = "SHOOT";
        else if (targetShooterRPM > 1000) reqState = "FLYWHEEL";
        else if (isIntakeActive) reqState = hasCaughtObject ? "CAUGHT" : "INTAKING";
        else reqState = "CRUISE";

        if (!reqState.equals(lastLedState)) lastLedState = reqState;
    }

    public void setLightAnimation(double min, double max, double durationSec) {
        if (targetMin == min && targetMax == max && Math.abs(animationDurationSec - durationSec) < 0.01) return;
        this.targetMin = min;
        this.targetMax = max;
        this.animationDurationSec = durationSec;
        this.transitionFromPos = lightLeft.getPosition();
        this.transitionToPos = this.targetMin;
        this.currentLightState = LightState.TRANSITIONING;
        this.lightTimer.reset();
    }

    public void updateLights() {
        double currentPosition = lightLeft.getPosition();
        double elapsedTime = lightTimer.seconds();
        switch (currentLightState) {
            case IDLE: break;
            case TRANSITIONING:
                double transitionProgress = Math.min(elapsedTime / TRANSITION_DURATION_SEC, 1.0);
                currentPosition = transitionFromPos + (transitionToPos - transitionFromPos) * transitionProgress;
                if (transitionProgress >= 1.0) {
                    currentPosition = transitionToPos;
                    animationMin = targetMin;
                    animationMax = targetMax;
                    isFadingUp = true;
                    currentLightState = LightState.ANIMATING;
                    lightTimer.reset();
                }
                break;
            case ANIMATING:
                double animationProgress = Math.min(elapsedTime / animationDurationSec, 1.0);
                if (isFadingUp) currentPosition = animationMin + (animationMax - animationMin) * animationProgress;
                else currentPosition = animationMax - (animationMax - animationMin) * animationProgress;
                if (animationProgress >= 1.0) {
                    isFadingUp = !isFadingUp;
                    lightTimer.reset();
                }
                break;
        }
        lightLeft.setPosition(currentPosition);
        lightRight.setPosition(currentPosition);
    }

    // --- 路径构建 ---
    public void buildPaths() {
        // Path 1 - Path 13 保持不变
        Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(27.495, 132.350), new Pose(55.951, 93.204)))
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-50)).build();

        Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(55.951, 93.204), new Pose(46.602, 87.500)))
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180)).build();

        Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(46.602, 87.500), new Pose(24.000, 87.500)))
                .setTangentHeadingInterpolation().build();

        Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(24.000, 87.500), new Pose(28.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90)).build();

        Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(28.000, 80.000), new Pose(21.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90)).build();

        Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.000, 80.000), new Pose(56.000, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-51)).build();

        Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 80.500), new Pose(44.505, 66.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-51), Math.toRadians(180)).build();

        Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(44.505, 66.000), new Pose(18.000, 66.000)))
                .setTangentHeadingInterpolation().build();

        Path9 = follower.pathBuilder().addPath(new BezierCurve(new Pose(18.000, 66.000), new Pose(29.500, 60.000), new Pose(28.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90)).build();

        Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(28.000, 80.000), new Pose(21.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90)).build();

        Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.000, 80.000), new Pose(56.000, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-51)).build();

        Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 80.500), new Pose(43.100, 44.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-51), Math.toRadians(180)).build();

        Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(43.100, 44.000), new Pose(18.000, 44.000)))
                .setTangentHeadingInterpolation().build();

        // 【修改点】Path 14: 直接从第三排吸取点去射击点
        // 起点 (Path13终点): (18, 44)
        // 终点 (射击位): (56, 80.5)
        // 角度: 180 -> -51
        Path14 = follower.pathBuilder().addPath(new BezierLine(new Pose(18.000, 44.000), new Pose(56.000, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-51)).build();

        // Path 15 (开门) 和 Path 16 (从门去射击) 已删除

        // Path 17: 射击点 -> 停靠
        Path17 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 80.500), new Pose(56.000, 62.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-51), Math.toRadians(180)).build();
    }
}