package org.firstinspires.ftc.teamcode.auto.cpx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * 带有循环逻辑的底盘控制程序
 * 逻辑：Path 4 -> [Path 5 -> Path 6 循环 15秒] -> Path 7 ...
 */
@Autonomous(name = "Test: 循环路径测试 (15s中断)", group = "Test")
public class ChassisCycleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer cycleTimer; // 新增：用于循环计时的计时器
    private int pathState = 0;

    // 起始姿态
    private final Pose startPose = new Pose(27.495, 132.350, Math.toRadians(-36));

    // 路径对象
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
    private PathChain Path8, Path9, Path10, Path11, Path12, Path13;

    // 模拟动作停顿时间
    private final long waitTime = 300;

    @Override
    public void init() {
        pathTimer = new Timer();
        cycleTimer = new Timer(); // 初始化循环计时器

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        // 显示循环剩余时间（仅在循环状态下有意义）
        if (pathState >= 50 && pathState <= 53) {
            telemetry.addData("Cycle Timer", "%.1f / 15.0s", cycleTimer.getElapsedTimeSeconds());
        }
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // 执行 Path 1
                follower.followPath(Path1, true);
                setPathState(1);
                break;
            case 1: // 等待 Path 1
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;
            case 2: // 等待 Path 2
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;
            case 3: // 等待 Path 3
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path4, true); // 去往循环起始点 (右侧)
                    setPathState(4);
                }
                break;

            case 4: // 等待 Path 4 完成 -> 准备进入循环
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    // --- 进入循环逻辑 ---
                    cycleTimer.resetTimer(); // 重置 15秒 计时器
                    setPathState(50); // 跳转到循环决策状态
                }
                break;

            // ================= 循环逻辑块 Start =================
            case 50: // 循环决策点：在右侧 (Path 4 终点 / Path 6 终点)
                // 检查：如果时间超过 15秒，跳出循环，去 Path 7
                if (cycleTimer.getElapsedTimeSeconds() > 15.0) {
                    setPathState(7); // 立即“截断”，跳转到 Path 7 逻辑
                } else {
                    // 时间没到，继续循环：执行 Path 5 (去左侧)
                    follower.followPath(Path5, true);
                    setPathState(51);
                }
                break;

            case 51: // 等待 Path 5 (到达左侧)
                if (!follower.isBusy()) {
                    // 可以在这里加吸取动作逻辑
                    // 动作完成后，执行 Path 6 (回右侧)
                    if (pathTimer.getElapsedTime() > waitTime) {
                        follower.followPath(Path6, true);
                        setPathState(52);
                    }
                }
                break;

            case 52: // 等待 Path 6 (回到右侧)
                if (!follower.isBusy()) {
                    // 可以在这里加发射动作逻辑
                    // 动作完成后，回到决策点 State 50 检查时间
                    if (pathTimer.getElapsedTime() > waitTime) {
                        setPathState(50);
                    }
                }
                break;
            // ================= 循环逻辑块 End =================

            // 注意：这里没有 Case 5 和 Case 6 了，被上面的 50-52 替代

            case 7: // 开始 Path 7 (从循环结束点接续)
                // 此时机器人一定位于 Path 6 的终点 (57.786, 85.049)，正好是 Path 7 的起点
                follower.followPath(Path7, true);
                setPathState(8);
                break;

            case 8: // 等待 Path 7 (后续逻辑照旧)
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path8, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path9, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path10, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path11, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path12, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > waitTime) {
                    follower.followPath(Path13, true);
                    setPathState(14);
                }
                break;

            case 14: // 结束
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        // ... (Path 1-4 保持不变)
        Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(27.495, 132.350), new Pose(58.951, 93.204))).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-39)).build();
        Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(58.951, 93.204), new Pose(44.505, 59.600))).setLinearHeadingInterpolation(Math.toRadians(-39), Math.toRadians(180)).build();
        Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(44.505, 59.600), new Pose(12.500, 59.650))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
        Path4 = follower.pathBuilder().addPath(new BezierCurve(new Pose(12.500, 59.650), new Pose(41.942, 55.922), new Pose(57.786, 85.049))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45)).build();

        // 循环路径段 (与你提供的保持一致)
        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(57.786, 85.049), // 循环起点 (右)
                        new Pose(41.942, 55.922),
                        new Pose(12.117, 61.282)  // 循环折返点 (左)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(140))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(12.117, 61.282), // 从左
                        new Pose(41.942, 55.922),
                        new Pose(57.786, 85.049)  // 回右
                ))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(-45))
                .build();

        // 后续路径 (Path 7 起点必须接在 Path 6 终点)
        Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(57.786, 85.049), new Pose(46.602, 84.300))).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180)).build();

        // ... (Path 8-13 保持不变)
        Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(46.602, 84.300), new Pose(22.000, 84.300))).setTangentHeadingInterpolation().build();
        Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(22.000, 84.300), new Pose(57.786, 85.049))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45)).build();
        Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(57.786, 85.049), new Pose(43.107, 35.417))).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180)).build();
        Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(43.107, 35.417), new Pose(12.500, 35.417))).setTangentHeadingInterpolation().build();
        Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.500, 35.417), new Pose(57.553, 85.515))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45)).build();
        Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(57.553, 85.515), new Pose(57.553, 56.388))).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180)).build();
    }
}