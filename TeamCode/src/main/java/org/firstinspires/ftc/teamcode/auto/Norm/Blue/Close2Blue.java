package org.firstinspires.ftc.teamcode.auto.Norm.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor; // 新增
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // 新增
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "近点 两排 蓝方", group = "A")
public class Close2Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // 状态机变量
    private int pathState = 0;

    // --- 新增：防走火状态锁 ---
    private boolean isMozartBraked = false;

    // 硬件定义
    private DcMotorEx SH, MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private Servo LP, RP;

    private DistanceSensor distanceSensor2;

    // 路径定义 (移除了 Cycle 3 相关的路径)
    private PathChain path1_Preload;
    private PathChain path2_ToObelisk;
    private PathChain path3_Intake1;
    private PathChain path4_Maneuver;
    private PathChain path5_Score1;
    private PathChain path6_ToSpike2;
    private PathChain path7_Intake2;
    private PathChain path8_Score2;
    private PathChain path12_Park; // 直接接停车

    // 常量配置
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    PathConstraints slowConstraints = new PathConstraints(30.0, 10.0, 1.0, 1.0);
    // 定义起始姿态
    private final Pose startPose = new Pose(33.600, 135.560, Math.toRadians(270));

    public void buildPaths() {
        // Path 1: Preload
        path1_Preload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.600, 135.560), new Pose(48.500, 95.900)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-48))
                .build();

        // Path 2: 观测
        path2_ToObelisk = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(48.500, 95.900), new Pose(60, 83.170), new Pose(42.400, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 3: 吸取 1
        path3_Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.400, 84.000), new Pose(22, 83.644)))
                .setConstraints(slowConstraints)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: 推闸/机动
        path4_Maneuver = follower.pathBuilder()
                .setConstraints(slowConstraints)
                .addPath(new BezierCurve(new Pose(22, 83.644), new Pose(30.000, 77.000), new Pose(18.000, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: 发射 Cycle 1
        path5_Score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18, 75.000), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 6: 准备吸第二排
        path6_ToSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.900, 84.00), new Pose(48.000, 63.5)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 7: 吸取 2
        path7_Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 63.5), new Pose(12.500, 63.5)))
                .setConstraints(slowConstraints)
                .setTangentHeadingInterpolation()
                .build();

        // Path 8: 发射 Cycle 2
        path8_Score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.5000, 63.5), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 12: 停车 (Path 9, 10, 11 已移除)
        // 注意：起点坐标与 Path 8 终点基本重合，Follower 会自动处理微小误差
        path12_Park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.760, 83.760), new Pose(28.000, 70.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        // --- 硬件初始化 ---
        SH = hardwareMap.get(DcMotorEx.class, "SH");
        MOZART = hardwareMap.get(DcMotorEx.class, "MOZART");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        RP = hardwareMap.get(Servo.class, "RP");
        LP = hardwareMap.get(Servo.class, "LP");

        washer = hardwareMap.get(CRServo.class, "washer");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        ClassifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");

        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "juju2");

        // --- 电机配置 ---
        SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfOrig = new PIDFCoefficients(250, 0.1, 30, 13);
        SH.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfOrig);

        MOZART.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        LP.setPosition(0.3546);
        RP.setPosition(0.667);

        // --- Follower 初始化 ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Close2CO Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("SH RPM", getShooterRPM());
        telemetry.addData("Mozart Braked", isMozartBraked);

        if (distanceSensor2 != null) {
            telemetry.addData("D2", "%.1f", distanceSensor2.getDistance(DistanceUnit.MM));
        }

        telemetry.update();
    }

    // --- 状态机逻辑 ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // 开始 Path 1 (Preload)
                follower.followPath(path1_Preload, true);
                SH.setVelocity(2500);
                setPathState(1);
                break;

            case 1: // 等待 Path 1 完成 -> 发射预载
                if (!follower.isBusy()) {
                    sleep(150);
                    runShooterLogic(2550);
                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        stopShooting();
                        setPathState(2);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 2: // 开始 Path 2 (观测)
                follower.followPath(path2_ToObelisk, true);
                setPathState(3);
                break;

            case 3: // 等待 Path 2 完成
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4: // 开始 Path 3 (吸取 1)
                isMozartBraked = false;
                follower.setMaxPower(0.9);
                follower.followPath(path3_Intake1, false);
                setPathState(5);
                break;

            case 5: // Path 3 运行中
                runIntakeLogic();
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // 开始 Path 4 (推闸/机动)
                follower.setMaxPower(1);
                stopIntake();
                follower.followPath(path4_Maneuver, true);
                setPathState(7);
                break;

            case 7: // 等待 Path 4 完成
                if (!follower.isBusy()) {
                    LP.setPosition(0.8156);
                    RP.setPosition(0.262);
                    SH.setVelocity(2500);
                    setPathState(8);
                }
                break;

            case 8: // 开始 Path 5 (回分)
                follower.followPath(path5_Score1, true);
                setPathState(9);
                break;

            case 9: // 等待 Path 5 完成 -> 发射 Cycle 1
                if (!follower.isBusy()) {
                    sleep(150);
                    runShooterLogic(2750);
                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        stopShooting();
                        setPathState(10);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 10: // 开始 Path 6 (去 Spike 2)
                follower.followPath(path6_ToSpike2, true);
                setPathState(11);
                break;

            case 11: // 等待 Path 6 完成
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;

            case 12: // 开始 Path 7 (吸取 2)
                isMozartBraked = false;
                follower.setMaxPower(0.7);
                follower.followPath(path7_Intake2, false);
                setPathState(13);
                break;

            case 13: // Path 7 运行中
                runIntakeLogic();
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14: // 开始 Path 8 (回分 Cycle 2)
                stopIntake();
                follower.setMaxPower(1);
                follower.followPath(path8_Score2, true);
                SH.setVelocity(2500);
                setPathState(15);
                break;

            case 15: // 等待 Path 8 完成 -> 发射 Cycle 2
                if (!follower.isBusy()) {
                    sleep(150);
                    runShooterLogic(2750);
                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        stopShooting();
                        // --- 关键修改：发射完 Cycle 2 后直接去停车 ---
                        setPathState(16);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            // --- 移除了 Cycle 3 相关的 Case (16-21) ---

            case 16: // 开始 Path 12 (停车)
                // 这里对应原来的 Path 12
                follower.followPath(path12_Park, true);
                setPathState(17);
                break;

            case 17: // 结束
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    // --- 功能函数 ---

    private void runShooterLogic(double targetRPM) {
        double targetVelocity = (targetRPM * TICKS_PER_REV * GEAR_RATIO) / 60.0;
        SH.setVelocity(targetVelocity);

        double currentRPM = getShooterRPM();

        if (Math.abs(currentRPM - targetRPM) <= 100) {
            MOZART.setPower(1.0);
            Hold.setPower(-1.0);
            ClassifyServo.setPower(1.0);
            washer.setPower(1);
        } else {
            MOZART.setPower(0);
        }
    }

    private void stopShooting() {
        SH.setPower(0);
        MOZART.setPower(0);
        Hold.setPower(0);
        ClassifyServo.setPower(0);
        washer.setPower(0);
    }

    private double getShooterRPM() {
        return (SH.getVelocity() * 60.0) / (TICKS_PER_REV * GEAR_RATIO);
    }

    private void runIntakeLogic() {
        Intake.setPower(1.0);
        washer.setPower(1.0);
        Hold.setPower(1.0);
        ClassifyServo.setPower(1.0);

        double dist2 = distanceSensor2.getDistance(DistanceUnit.MM);

        if (!isMozartBraked) {
            if (dist2 < 50) {
                isMozartBraked = true;
            }
        }

        if (isMozartBraked) {
            MOZART.setPower(0.0);
        } else {
            MOZART.setPower(1.0);
        }
    }

    private void stopIntake() {
        Intake.setPower(0);
        washer.setPower(0);
        Hold.setPower(0);
        ClassifyServo.setPower(0);
        MOZART.setPower(0);
    }
    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}