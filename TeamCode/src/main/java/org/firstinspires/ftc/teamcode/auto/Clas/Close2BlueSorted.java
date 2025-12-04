package org.firstinspires.ftc.teamcode.Auto.Clas;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AutoSortingSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.Class.ColorClassifier;

@Autonomous(name = "近点 两排 蓝方 (带分类)", group = "PedroPathing")
public class Close2BlueSorted extends LinearOpMode {

    // =========================================================
    // 分类预设 (根据场上实际情况修改这里)
    // =========================================================
    // 第一排 (Path 3) 的情况
    private static final String INPUT_PATTERN_1 = "GPP";

    // 第二排 (Path 7) 的情况
    private static final String INPUT_PATTERN_2 = "PGP";
    private static final String TARGET_PATTERN = "GPP";

    // =========================================================
    // 硬件与变量
    // =========================================================
    private Follower follower;
    private AutoSortingSubsystem sorter;
    private ColorClassifier classifier;

    private DcMotorEx SH, MOZART;
    private Servo LP, RP;

    // 路径定义
    private PathChain path1_Preload;
    private PathChain path2_ToObelisk;
    private PathChain path3_Intake1;
    private PathChain path4_Maneuver;
    private PathChain path5_Score1;
    private PathChain path6_ToSpike2;
    private PathChain path7_Intake2;
    private PathChain path8_Score2;
    private PathChain path12_Park; // 停车

    private final Pose startPose = new Pose(33.600, 135.560, Math.toRadians(270));
    PathConstraints slowConstraints = new PathConstraints(30.0, 10.0, 1.0, 1.0);

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 1. 硬件初始化 ---
        SH = hardwareMap.get(DcMotorEx.class, "SH");
        MOZART = hardwareMap.get(DcMotorEx.class, "MOZART");
        RP = hardwareMap.get(Servo.class, "RP");
        LP = hardwareMap.get(Servo.class, "LP");

        // 电机配置
        SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SH.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(250, 0.1, 30, 13));
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        LP.setPosition(0.3546);
        RP.setPosition(0.667);

        // --- 2. Follower & Subsystem 初始化 ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 视觉初始化
        classifier = new ColorClassifier();
        classifier.init(hardwareMap, "ClassifyCam", telemetry);

        // 分类子系统初始化 (传入 Follower 以便控制底盘)
        sorter = new AutoSortingSubsystem(hardwareMap, telemetry, classifier, this::opModeIsActive, follower);

        buildPaths();

        telemetry.addData("Status", "Close2Blue Sorted Initialized");
        telemetry.update();

        waitForStart();
        classifier.setEnabled(true);

        if (opModeIsActive()) {
            // =================================================
            // 0. Preload Cycle (预载)
            // =================================================
            runShooter(2350);
            follower.followPath(path1_Preload, true);
            waitPath();

            shoot(2000); // 发射

            // =================================================
            // 1. First Sample Cycle (第一排)
            // =================================================
            follower.followPath(path2_ToObelisk, true);
            waitPath();

            // 吸取路径
            follower.setMaxPower(0.9);
            follower.followPath(path3_Intake1, false);
            waitPath();

            // *** 执行分类逻辑 1 ***
            // 此时机器人位于第一排终点
            sorter.executeStrategy(INPUT_PATTERN_1, TARGET_PATTERN);

            sorter.stopIntake(); // 停止吸取
            follower.setMaxPower(1.0);

            // 机动避让 + 举升
            follower.followPath(path4_Maneuver, true);
            waitPath();

            LP.setPosition(0.8156);
            RP.setPosition(0.262);
            runShooter(2550);

            // 移动到发射位
            follower.followPath(path5_Score1, true);
            waitPath();
            shoot(2500);

            // =================================================
            // 2. Second Sample Cycle (第二排)
            // =================================================
            follower.followPath(path6_ToSpike2, true);
            waitPath();

            // 吸取路径
            follower.setMaxPower(0.7);
            follower.followPath(path7_Intake2, false);
            waitPath();

            // *** 执行分类逻辑 2 ***
            sorter.executeStrategy(INPUT_PATTERN_2, TARGET_PATTERN);

            sorter.stopIntake();
            follower.setMaxPower(1.0);

            // 移动到发射位
            follower.followPath(path8_Score2, true);
            runShooter(2550);
            waitPath();
            shoot(2500);

            // =================================================
            // 3. Park (停车) - 直接接在 Cycle 2 之后
            // =================================================
            SH.setPower(0); // 关闭摩擦轮
            follower.followPath(path12_Park, true);
            waitPath();
        }

        // 结束
        classifier.close();
    }

    // --- 辅助函数 ---

    /**
     * 阻塞等待路径完成，同时更新 Follower
     */
    private void waitPath() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    /**
     * 设置摩擦轮转速
     */
    private void runShooter(double rpm) {
        double velocity = (rpm * 28.0) / 60.0;
        SH.setVelocity(velocity);
    }

    /**
     * 发射动作 (阻塞式)
     * 开启整个 Intake 系统 (Mozart/Washer/Hold) 将球送入
     */
    private void shoot(long durationMs) {
        long start = System.currentTimeMillis();

        // 使用 Subsystem 的方法开启所有传送机构
        sorter.setIntakeMode();

        while (opModeIsActive() && (System.currentTimeMillis() - start < durationMs)) {
            follower.update(); // 保持定位更新
        }
        sorter.stopIntake();
    }

    // --- 路径构建 ---
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

        // Path 3: Intake 1
        path3_Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(40.400, 84.000), new Pose(18, 83.644)))
                .setConstraints(slowConstraints)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: Maneuver (推闸/避让)
        path4_Maneuver = follower.pathBuilder()
                .setConstraints(slowConstraints)
                .addPath(new BezierCurve(new Pose(19.840, 83.644), new Pose(30.000, 77.000), new Pose(18.000, 73.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: Score 1
        path5_Score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15.200, 74.000), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 6: To Spike 2
        path6_ToSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.900, 83.900), new Pose(48.000, 62.640)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 7: Intake 2
        path7_Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 62.640), new Pose(8.500, 59.640)))
                .setConstraints(slowConstraints)
                .setTangentHeadingInterpolation()
                .build();

        // Path 8: Score 2
        path8_Score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.000, 59.640), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 12: Park (直接从 Score 2 的终点开始)
        // Score 2 End: (59.900, 84.000)
        // Park Start: (59.760, 83.760) -> 足够接近，Follower 会自动处理
        path12_Park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.760, 83.760), new Pose(28.000, 70.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(270))
                .build();
    }
}