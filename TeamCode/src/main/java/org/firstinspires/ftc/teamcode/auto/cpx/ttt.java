package org.firstinspires.ftc.teamcode.auto.cpx;

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

@Autonomous(name = "Final: 13段路径全动作自动程序", group = "Main")
public class ttt extends OpMode {

    private Follower follower;
    private Timer pathTimer, cycleTimer, actionTimer;
    private int pathState = 0;

    // --- 配置常量 ---
    private final double FirstShootingPower = 2800.0;
    private final double NormalShootingPower = 2400.0;
    private final long waitTime = 100; // 极短的逻辑衔接缓冲

    // --- 硬件定义 ---
    private DcMotorEx SH, HS, Intake, Mozart;
    private CRServo Hold;
    private Servo servoRP = null;
    private Servo servoLP = null;
    private DigitalChannel juju;

    // --- 飞轮 PID 变量 ---
    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double targetShooterRPM = 0;
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.006, SHOOTER_F = 0.0004, SHOOTER_D = 0.00001, TICKS_PER_REV = 28.0;

    // --- 机构状态控制 ---
    private boolean isIntakeActive = false;
    private boolean hasCaughtObject = false;
    private ElapsedTime shootActionTimer = new ElapsedTime();
    private double shootTimeLimitSec = 0;
    private boolean isShootingTaskActive = false;
    private boolean isChassisPausedByShot = false;

    // --- 路径定义 ---
    private final Pose startPose = new Pose(27.495, 132.350, Math.toRadians(-36));
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13;

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
        juju = hardwareMap.get(DigitalChannel.class, "juju");
        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");
        juju.setMode(DigitalChannel.Mode.INPUT);
        Mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        servoLP.setPosition(0.60);
        servoRP.setPosition(0.325);
        buildPaths();
    }

    @Override
    public void start() { setPathState(0); }

    @Override
    public void loop() {
        updateShooterPID();    // PID 始终最高优先级
        if (isShootingTaskActive) runShootingSequence(); else updateIntakeLogic();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("RPM", getShooterRPM());
        telemetry.addData("Loop Time", cycleTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    // ================= 状态机逻辑 =================

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path 1: 启动与跑打预备
                targetShooterRPM = FirstShootingPower;
                follower.followPath(Path1, false); // 不 holdEnd
                setPathState(1);
                break;

            case 1: // Path 1 运行中：0.4s后射击0.3s
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(1145);
                }
                break;
            case 1145:
                if (!isShootingTaskActive) {
                    follower.followPath(Path2, true); // Path 2 holdEnd
                    setPathState(2);
                }
                break;

            case 2: // Path 2 结束 -> Path 3 开启 Intake
                if (!follower.isBusy()) {
                    setIntake(true);
                    servoLP.setPosition(0.30);
                    servoRP.setPosition(0.625);
                    follower.followPath(Path3, false); // Path 3 不 holdEnd
                    setPathState(3);
                }
                break;

            case 3: // Path 3 结束 -> Path 4
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower; // 设置飞轮速度
                    follower.followPath(Path4, true); // Path 4 holdEnd，为了最后定点发射
                    setPathState(4);
                }
                break;

            case 4: // Path 4 结束 -> 定点发射并进入 15s 循环
                if (!follower.isBusy()) {
                    shooting(500, false); // 定点发射 0.2s
                    setPathState(49); // 进入循环准备
                }
                break;

            case 49:
                if (!isShootingTaskActive) {
                    cycleTimer.resetTimer(); // 开启 15s 计时
                    setPathState(50); // 进入循环决策
                }
                break;

            // ================= 15秒循环逻辑 =================
            case 50:
                if (cycleTimer.getElapsedTimeSeconds() >= 10.0) {
                    setIntake(false); // 循环结束，虽然你没提，但逻辑上建议关闭
                    setPathState(7); // 跳出循环执行 Path 7
                } else {
                    setIntake(true); // 循环期间一直开启
                    follower.followPath(Path5, true);
                    setPathState(51);
                }
                break;

            case 51: // Path 5 左侧：等待 1s intake
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                        follower.followPath(Path6, true);
                        setPathState(52);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 52: // Path 6 右侧：定点发射 0.2s
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(53);
                }
                break;

            case 53:
                if (!isShootingTaskActive) setPathState(50); // 射完一圈，回决策点
                break;
            // ================= 循环结束 =================

            case 7: // Path 7
                follower.followPath(Path7, true);
                setPathState(8);
                break;

            case 8: // Path 7/8 交接开启 Intake
                if (!follower.isBusy()) {
                    setIntake(true);
                    follower.followPath(Path8, false); // 不 holdEnd
                    setPathState(9);
                }
                break;

            case 9: // 返回发射位置
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.followPath(Path9, true);
                    setPathState(10);
                }
                break;

            case 10: // 定点发射 0.2s -> 执行 Path 10
                if (!follower.isBusy()) {
                    if (!isShootingTaskActive && actionTimer.getElapsedTimeSeconds() > 0.5) {
                        shooting(500, false);
                        actionTimer.resetTimer();
                    }
                    if (!isShootingTaskActive && actionTimer.getElapsedTimeSeconds() > 0.3) {
                        follower.followPath(Path10, true);
                        setPathState(11);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 11: // Path 10/11 交接开启 Intake
                if (!follower.isBusy()) {
                    setIntake(true);
                    follower.followPath(Path11, false); // 不 holdEnd
                    setPathState(12);
                }
                break;

            case 12: // 返回发射位置执行最后的 0.2s 发射
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.followPath(Path12, true);
                    setPathState(13);
                }
                break;

            case 13: // 最后的发射 -> Path 13 (全停)
                if (!follower.isBusy()) {
                    if (!isShootingTaskActive && actionTimer.getElapsedTimeSeconds() > 0.5) {
                        shooting(500, false);
                        actionTimer.resetTimer();
                    }
                    if (!isShootingTaskActive && actionTimer.getElapsedTimeSeconds() > 0.3) {
                        // 停止所有结构动作
                        setIntake(false);
                        targetShooterRPM = 0;
                        follower.followPath(Path13, true);
                        setPathState(14);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 14:
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    // ================= 功能子系统 =================

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
            Intake.setPower(1.0); Mozart.setPower(0.8); Hold.setPower(1.0);
        } else {
            isShootingTaskActive = false;
            Intake.setPower(0); Mozart.setPower(0); Hold.setPower(0);
            if (isChassisPausedByShot) follower.resumePathFollowing();
        }
    }
    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
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

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(27.495, 132.350), new Pose(58.951, 93.204))).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-45)).build();

        Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(58.951, 93.204), new Pose(44.505, 63.5))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
        Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(44.505, 63.5), new Pose(12.500, 63.5))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        Path4 = follower.pathBuilder().addPath(new BezierCurve(new Pose(12.500, 63.5), new Pose(41.942, 45), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48)).build();
        Path5 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60, 84), new Pose(43, 15), new Pose(7, 62.5))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(150)).build();
        Path6 = follower.pathBuilder().addPath(new BezierCurve(new Pose(7.5, 62.5), new Pose(41.942, 45), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(-48)).build();

        Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 84), new Pose(46.602, 85))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
        Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(46.602, 85), new Pose(22.000, 85))).setTangentHeadingInterpolation().build();

        Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(22.000, 85), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48)).build();

        Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 84), new Pose(43.107, 38))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
        Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(43.107, 38), new Pose(12.500, 38))).setTangentHeadingInterpolation().build();

        Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.500, 38), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48)).build();
        Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 84), new Pose(57.553, 56.388))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
    }
}