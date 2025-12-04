package org.firstinspires.ftc.teamcode.Auto.Norm.Blue;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "近点 三排 蓝方", group = "PedroPathing")
public class Close3Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // 状态机变量
    private int pathState = 0;

    // --- 新增：防走火状态锁 ---
    private boolean isMozartBraked = false;

    // 硬件定义 (修改 ColorSensor 为 RevColorSensorV3)
    private DcMotorEx SH, MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private Servo LP, RP;
    private RevColorSensorV3 color;

    // 路径定义
    private PathChain path1_Preload;
    private PathChain path2_ToObelisk;
    private PathChain path3_Intake1;
    private PathChain path4_Maneuver;
    private PathChain path5_Score1;
    private PathChain path6_ToSpike2;
    private PathChain path7_Intake2;
    private PathChain path8_Score2;
    private PathChain path9_ToSpike3;
    private PathChain path10_Intake3;
    private PathChain path11_Score3;
    private PathChain path12_Park;

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
                .addPath(new BezierLine(new Pose(40.400, 84.000), new Pose(18, 83.644)))
                .setConstraints(slowConstraints)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: 推闸/机动
        path4_Maneuver = follower.pathBuilder()
                .setConstraints(slowConstraints)
                .addPath(new BezierCurve(new Pose(19.840, 83.644), new Pose(30.000, 77.000), new Pose(18.000, 73.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: 发射 Cycle 1
        path5_Score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15.200, 74.000), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 6: 准备吸第二排
        path6_ToSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.900, 83.900), new Pose(48.000, 62.640)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 7: 吸取 2
        path7_Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 62.640), new Pose(8.500, 59.640)))
                .setConstraints(slowConstraints)
                .setTangentHeadingInterpolation()
                .build();

        // Path 8: 发射 Cycle 2
        path8_Score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.000, 59.640), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 9: 准备吸第三排
        path9_ToSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.880, 84.000), new Pose(48.000, 40.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 10: 吸取 3
        path10_Intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(41.000, 40.000), new Pose(8.500, 40.500)))
                .setConstraints(slowConstraints)
                .setTangentHeadingInterpolation()
                .build();

        // Path 11: 发射 Cycle 3
        path11_Score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(21.500, 35.500), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 12: 停车
        path12_Park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.760, 83.760), new Pose(28.000, 70.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(270))
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

        // 修改为 RevColorSensorV3
        color = hardwareMap.get(RevColorSensorV3.class, "color");

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

        telemetry.addData("Status", "Close3CO Initialized");
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

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("SH RPM", getShooterRPM());
        telemetry.addData("Mozart Braked", isMozartBraked); // 调试信息
        telemetry.update();
    }

    // --- 状态机逻辑 ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // 开始 Path 1 (Preload)
                follower.followPath(path1_Preload, true);
                SH.setVelocity(2350);
                setPathState(1);
                break;

            case 1: // 等待 Path 1 完成 -> 发射预载
                if (!follower.isBusy()) {
                    runShooterLogic(2350);
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
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
                // --- 关键修改：进入吸取状态前重置防走火标志 ---
                isMozartBraked = false;
                follower.setMaxPower(0.9);
                follower.followPath(path3_Intake1, false);
                setPathState(5);
                break;

            case 5: // Path 3 运行中
                runIntakeLogic(); // 执行吸取逻辑
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
                    SH.setVelocity(2550);
                    setPathState(8);
                }
                break;

            case 8: // 开始 Path 5 (回分)
                follower.followPath(path5_Score1, true);
                setPathState(9);
                break;

            case 9: // 等待 Path 5 完成 -> 发射 Cycle 1
                if (!follower.isBusy()) {
                    runShooterLogic(2550);
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
                // --- 关键修改：重置标志 ---
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
                SH.setVelocity(2550);
                setPathState(15);
                break;

            case 15: // 等待 Path 8 完成 -> 发射 Cycle 2
                if (!follower.isBusy()) {
                    runShooterLogic(2550);
                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        stopShooting();
                        setPathState(16);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 16: // 开始 Path 9 (去 Spike 3)
                follower.followPath(path9_ToSpike3, true);
                setPathState(17);
                break;

            case 17: // 等待 Path 9
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;

            case 18: // 开始 Path 10 (吸取 3)
                // --- 关键修改：重置标志 ---
                isMozartBraked = false;
                follower.setMaxPower(0.7);
                follower.followPath(path10_Intake3, false);
                setPathState(19);
                break;

            case 19: // Path 10 运行中
                runIntakeLogic();
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;

            case 20: // 开始 Path 11 (回分 Cycle 3)
                stopIntake();
                follower.setMaxPower(1);
                follower.followPath(path11_Score3, true);
                SH.setVelocity(2550);
                setPathState(21);
                break;

            case 21: // 等待 Path 11 完成 -> 发射 Cycle 3
                if (!follower.isBusy()) {
                    runShooterLogic(2550);
                    if (actionTimer.getElapsedTimeSeconds() > 2.5) {
                        stopShooting();
                        setPathState(22);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 22: // 开始 Path 12 (停车)
                follower.followPath(path12_Park, true);
                setPathState(23);
                break;

            case 23: // 结束
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

        if (Math.abs(currentRPM - targetRPM) <= 50) {
            MOZART.setPower(1.0);
            Hold.setPower(1.0);
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

    /**
     * 吸取逻辑：使用 TeleOp 中正确的算法
     */
    private void runIntakeLogic() {
        // 1. 基础结构开启
        Intake.setPower(1.0);
        washer.setPower(1.0);
        Hold.setPower(1.0);
        ClassifyServo.setPower(1.0);

        // 2. 防走火检测 (使用 NormalizedRGBA 和 TeleOp 中的逻辑)
        NormalizedRGBA colors = color.getNormalizedColors();
        if (!isMozartBraked) {
            // TeleOp 逻辑：如果任一颜色通道显著（乘以255后大于1），认为检测到球
            if (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1) {
                isMozartBraked = true; // 锁定刹车状态
            }
        }

        // 3. 根据状态控制 MOZART
        if (isMozartBraked) {
            MOZART.setPower(0.0); // 停止
        } else {
            MOZART.setPower(1.0); // 运行
        }
    }

    private void stopIntake() {
        Intake.setPower(0);
        washer.setPower(0);
        Hold.setPower(0);
        ClassifyServo.setPower(0);
        MOZART.setPower(0);
    }
}