package org.firstinspires.ftc.teamcode.Auto;

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
 * 自动程序: Far1CO
 * 仅包含底盘路径跟随逻辑
 * 路径描述：包含10段混合了直线和曲线的复杂路径
 */
@Autonomous(name = "Far1CO", group = "PedroPathing")
public class Far1CO extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private PathChain mainPath;

    // 定义起始姿态：根据 Path 1 的起点 (56.2, 8.08) 和起始角度 (270度)
    private final Pose startPose = new Pose(56.200, 8.080, Math.toRadians(270));

    /**
     * 构建路径的方法
     * 完全复刻了 GeneratedPath 中的逻辑
     */
    public void buildPaths() {
        mainPath = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(56.200, 8.080), new Pose(57.800, 14.250))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-69))
                //处理预制

                .addPath(
                        // Path 2
                        new BezierLine(new Pose(57.800, 14.250), new Pose(40.280, 35.600))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-69), Math.toRadians(180))
                //准备吸球

                .addPath(
                        // Path 3
                        new BezierLine(new Pose(40.280, 35.600), new Pose(21.000, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                //吸完了

                .addPath(
                        // Path 4
                        new BezierLine(new Pose(21.000, 35.500), new Pose(57.800, 14.250))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-69))
                //舍了舍了

                .addPath(
                        // Path 5
                        new BezierLine(new Pose(57.800, 14.250), new Pose(11.000, 17.220))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-69), Math.toRadians(-156))
                //嘬一口

                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(11.000, 17.220),
                                new Pose(14.370, 14.250),
                                new Pose(11.160, 10.930)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(-156))
                //嘬两口

                .addPath(
                        // Path 7
                        new BezierCurve(
                                new Pose(11.160, 10.930),
                                new Pose(10.930, 12.590),
                                new Pose(10.700, 10.455)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(-106))
                //还想嘬

                .addPath(
                        // Path 8
                        new BezierCurve(
                                new Pose(10.700, 10.455),
                                new Pose(8.200, 13.400),
                                new Pose(8.550, 8.910)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-106), Math.toRadians(-90))
                //嘬三口

                .addPath(
                        // Path 9
                        new BezierCurve(
                                new Pose(8.550, 8.910),
                                new Pose(32.300, 14.000),
                                new Pose(57.800, 14.250)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-69))
                //舍了舍了

                .addPath(
                        // Path 10
                        new BezierCurve(
                                new Pose(57.800, 14.250),
                                new Pose(28.280, 13.540),
                                new Pose(21.380, 9.980)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-69), Math.toRadians(180))
                //让我看看。。。。

                .build();
    }

    /**
     * 初始化阶段
     */
    @Override
    public void init() {
        opmodeTimer = new Timer();

        // 1. 创建 Follower
        follower = Constants.createFollower(hardwareMap);

        // 2. 设置起始姿态
        follower.setStartingPose(startPose);

        // 3. 构建路径
        buildPaths();

        telemetry.addData("Status", "Far1CO Initialized");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    /**
     * 开始运行阶段
     */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        // 开始跟随路径，true 表示结束后保持位置
        follower.followPath(mainPath, true);
    }

    /**
     * 循环更新阶段
     */
    @Override
    public void loop() {
        // 必须调用 update
        follower.update();

        // 调试信息
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        if (follower.isBusy()) {
            telemetry.addData("Status", "Running Path...");
            // 如果你想看当前是在跑第几段路径（大约），可以查看 TValue
            // telemetry.addData("Path T", follower.getCurrentTValue());
        } else {
            telemetry.addData("Status", "Path Completed (Holding)");
        }

        telemetry.update();
    }
}