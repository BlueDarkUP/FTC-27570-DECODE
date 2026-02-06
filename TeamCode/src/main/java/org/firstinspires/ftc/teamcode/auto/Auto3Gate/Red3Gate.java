package org.firstinspires.ftc.teamcode.auto.Auto3Gate;

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

@Autonomous(name = "红方开三次门", group = "Main")
public class Red3Gate extends OpMode {

    private Follower follower;
    private Timer pathTimer, cycleTimer, actionTimer;
    private int pathState = 0;

    // --- 保持原有的射击参数 ---
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

    // --- 路径变量 (Path1 - Path17) ---
    // 镜像起始位置: X = 144 - 27.495 = 116.505, Heading = 180 - (-36) = 216
    private final Pose startPose = new Pose(116.505, 132.350, Math.toRadians(216));
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9,
            Path10, Path11, Path12, Path13, Path14, Path15, Path16, Path17;

    @Override
    public void init() {
        pathTimer = new Timer();
        cycleTimer = new Timer();
        actionTimer = new Timer();
        shooterPidTimer.reset();

        // 硬件初始化 (与原程序一致)
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
        // 红方使用红色系
        led.setWaveMode('A', 0xFF0000, 0xFFA500, 200, 60);

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
        // 红方初始灯光
        setLightAnimation(C_RED, C_ORANGE, 1.5);
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
            // 阶段 0: 预载射击 (Path 1)
            // ==========================================
            case 0:
                targetShooterRPM = FirstShootingPower;
                follower.followPath(Path1, false);
                setPathState(1);
                break;

            case 1: // 到达射击点，执行射击
                if (pathTimer.getElapsedTimeSeconds() > 0.45) {
                    setLightAnimation(C_RED, C_ORANGE, 0.5);
                    shooting(500, false);
                    servoLP.setPosition(0.30);
                    servoRP.setPosition(0.625);
                    hasCaughtObject = false;
                    setPathState(2);
                }
                break;

            case 2: // 等待射击完成
                if (!isShootingTaskActive) {
                    targetShooterRPM = IdleShootingPower;
                    setPathState(100);
                }
                break;

            // ==========================================
            // 阶段 1: 第一排样本
            // ==========================================
            case 100: // 准备第一排 (Path 2)
                setLightAnimation(C_SAGE, C_GREEN, 1);
                follower.followPath(Path2, false);
                setPathState(101);
                break;

            case 101: // 吸取第一排 (Path 3)
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

            case 102: // 准备开门 (Path 4)
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.setMaxPower(1.0);
                    follower.followPath(Path4, false);
                    setPathState(103);
                }
                break;

            case 103: // 开门动作 (Path 5)
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(Path5, false);
                    setPathState(104);
                }
                break;

            case 104: // 前往射击点 (Path 6)
                if (!follower.isBusy()) {
                    // 红方灯光改为蓝紫或保持一致均可，这里保持一致以便调试
                    Thread.sleep(350);
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.setMaxPower(1);
                    follower.followPath(Path6, true);
                    setPathState(105);
                }
                break;

            case 105: // 射击
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
            // 阶段 2: 第二排样本
            // ==========================================
            case 200: // 准备第二排 (Path 7)
                targetShooterRPM = IdleShootingPower;
                setLightAnimation(C_SAGE, C_GREEN, 1);
                follower.followPath(Path7, false);
                setPathState(201);
                break;

            case 201: // 吸取第二排 (Path 8)
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

            case 202: // 准备开门 (Path 9)
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.setMaxPower(1.0);
                    follower.followPath(Path9, false);
                    setPathState(203);
                }
                break;

            case 203: // 开门动作 (Path 10)
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(Path10, false);
                    setPathState(204);
                }
                break;

            case 204: // 前往射击点 (Path 11)
                if (!follower.isBusy()) {
                    Thread.sleep(350);
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.setMaxPower(1);
                    follower.followPath(Path11, true);
                    setPathState(205);
                }
                break;

            case 205: // 射击
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
            // 阶段 3: 第三排样本
            // ==========================================
            case 300: // 准备第三排 (Path 12)
                targetShooterRPM = IdleShootingPower;
                setLightAnimation(C_SAGE, C_GREEN, 1);
                follower.followPath(Path12, false);
                setPathState(301);
                break;

            case 301: // 吸取第三排 (Path 13)
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

            case 302: // 准备开门 (Path 14)
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.setMaxPower(1.0);
                    follower.followPath(Path14, false);
                    setPathState(303);
                }
                break;

            case 303: // 开门动作 (Path 15)
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(Path15, false);
                    setPathState(304);
                }
                break;

            case 304: // 前往射击点 (Path 16)
                if (!follower.isBusy()) {
                    Thread.sleep(350);
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.setMaxPower(1);
                    follower.followPath(Path16, true);
                    setPathState(305);
                }
                break;

            case 305: // 射击
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

    // --- 辅助方法 (保持不变) ---
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

    // --- 路径构建 (镜像+下移) ---
    public void buildPaths() {
        // Path 1: Start -> Shoot Pos
        // Blue: X=55.951, Y=93.204, H=-50
        // Red: X=88.049, Y=93.204 (射击位Y不变), H=230
        Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(116.505, 132.350), new Pose(88.049, 93.204)))
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(230)).build();

        // Path 2: Shoot Pos -> Prep Row 1
        // Blue: X=46.602, Y=86.5 -> Red: X=97.398, Y=78.5 (下移8)
        Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.049, 93.204), new Pose(97.398, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0)).build();

        // Path 3: Prep Row 1 -> Intake Row 1
        // Blue: X=27.0, Y=86.5 -> Red: X=117.0, Y=78.5
        Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(97.398, 80.500), new Pose(119.000, 80.500)))
                .setTangentHeadingInterpolation().build();

        // Path 4: Intake Row 1 -> Prep Door
        // Blue: (30, 86.5) -> (28, 74)
        // Red: (114, 78.5) -> (116, 66)
        // H: 180 -> -90 (270)
        // Red H: 0 -> 270 (270)
        Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(114.000, 80.500), new Pose(116.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270)).build();

        // Path 5: OPEN DOOR (Hit)
        // Blue: (28, 74) -> (22, 74)
        // Red: (116, 66) -> (122, 66)
        Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(116.000, 75.000), new Pose(122.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270)).build();

        // Path 6: Door -> Shoot Pos
        // Blue: (22, 74) -> (56, 80.5)
        // Red: (122, 66) -> (88, 80.5) (射击位Y保持80.5)
        Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(122.000, 75.000), new Pose(88.000, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(232.3)).build();

        // Path 7: Shoot Pos -> Prep Row 2
        // Blue: (56, 80.5) -> (44.505, 66)
        // Red: (88, 80.5) -> (99.495, 58) (66-8=58)
        Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 80.500), new Pose(99.495, 58.000)))
                .setLinearHeadingInterpolation(Math.toRadians(232.3), Math.toRadians(0)).build();

        // Path 8: Prep Row 2 -> Intake Row 2
        // Blue: (44.505, 66) -> (21, 66)
        // Red: (99.495, 58) -> (123, 58)
        Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(99.495, 58.000), new Pose(126.000, 58.000)))
                .setTangentHeadingInterpolation().build();

        // Path 9: Intake Row 2 -> Prep Door (Curved)
        // Blue: Start(28, 66), CP(23.5, 66), End(28, 74)
        // Red: Start(116, 58), CP(120.5, 58), End(116, 66)
        Path9 = follower.pathBuilder().addPath(new BezierCurve(new Pose(116.000, 58.000), new Pose(120.500, 58.000), new Pose(116.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270)).build();

        // Path 10: OPEN DOOR (Hit)
        // Blue: (28, 74) -> (22, 74)
        // Red: (116, 66) -> (122, 66)
        Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(116.000, 75.000), new Pose(122.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270)).build();

        // Path 11: Door -> Shoot Pos
        // Red: (122, 66) -> (88, 80.5)
        Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(122.000, 75.000), new Pose(88.000, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(232.3)).build();

        // Path 12: Shoot Pos -> Prep Row 3
        // Blue: (56, 80.5) -> (43.1, 44)
        // Red: (88, 80.5) -> (100.9, 36) (44-8=36)
        Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 80.500), new Pose(100.900, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(232.3), Math.toRadians(0)).build();

        // Path 13: Prep Row 3 -> Intake Row 3
        // Blue: (43.1, 44) -> (21, 44)
        // Red: (100.9, 36) -> (123, 36)
        Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(100.900, 36.000), new Pose(126.000, 36.000)))
                .setTangentHeadingInterpolation().build();

        // Path 14: Intake Row 3 -> Prep Door
        // Blue: (28, 44) -> (28, 74)
        // Red: (116, 36) -> (116, 66)
        Path14 = follower.pathBuilder().addPath(new BezierLine(new Pose(116.000, 36.000), new Pose(116.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270)).build();

        // Path 15: OPEN DOOR (Hit)
        // Red: (116, 66) -> (122, 66)
        Path15 = follower.pathBuilder().addPath(new BezierLine(new Pose(116.000, 75.000), new Pose(122.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270)).build();

        // Path 16: Door -> Shoot Pos
        // Red: (122, 66) -> (88, 80.5)
        Path16 = follower.pathBuilder().addPath(new BezierLine(new Pose(122.000, 75.000), new Pose(88.000, 80.500)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(232.3)).build();

        // Path 17: Shoot Pos -> Park
        // Blue: (56, 80.5) -> (56, 62)
        // Red: (88, 80.5) -> (88, 62)
        Path17 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 80.500), new Pose(88.000, 62.000)))
                .setLinearHeadingInterpolation(Math.toRadians(232.3), Math.toRadians(0)).build();
    }
}