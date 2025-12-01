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
 * 自动程序: Close3CO (Close Side - 3 Cycles + Park)
 * 仅包含底盘路径跟随逻辑
 */
@Autonomous(name = "Close3CO", group = "PedroPathing")
public class Close3CO extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private PathChain mainPath;

    // 定义起始姿态：根据 Path 1 的起点 (33.6, 135.56) 和角度 (270度/朝下)
    private final Pose startPose = new Pose(33.600, 135.560, Math.toRadians(270));

    /**
     * 构建包含所有12段路径的 PathChain
     */
    public void buildPaths() {
        mainPath = follower.pathBuilder()
                // --- Preload ---
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(33.600, 135.560), new Pose(48.500, 95.900))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-48))
                // 这段路径完成后发射预制

                // --- Cycle 1 ---
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(48.500, 95.900),
                                new Pose(54.180, 83.170),
                                new Pose(40.400, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                // 这段路径运行期间观测方尖碑的apriltag

                .addPath(
                        // Path 3
                        new BezierLine(new Pose(40.400, 84.000), new Pose(19.840, 83.644))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                //这段路径完成意味着已经吸完三个遗物

                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(19.840, 83.644),
                                new Pose(23.900, 75.000),
                                new Pose(15.200, 74.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                //这段路径完成意味着已经推闸

                .addPath(
                        // Path 5
                        new BezierLine(new Pose(15.200, 74.000), new Pose(59.900, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                //舍了舍了

                // --- Cycle 2 ---
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(59.900, 83.900), new Pose(40.000, 59.640))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                //准备吸第二排

                .addPath(
                        // Path 7
                        new BezierLine(new Pose(40.000, 59.640), new Pose(20.000, 59.640))
                )
                .setTangentHeadingInterpolation()
                //吸完第二排

                .addPath(
                        // Path 8
                        new BezierLine(new Pose(20.000, 59.640), new Pose(59.900, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                //舍了舍了

                // --- Cycle 3 (新增部分) ---
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(59.880, 84.000), new Pose(41.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                //准备吸第三排

                .addPath(
                        // Path 10
                        new BezierLine(new Pose(41.000, 35.000), new Pose(21.500, 35.500))
                )
                .setTangentHeadingInterpolation()
                //吸完第三排

                .addPath(
                        // Path 11
                        new BezierLine(new Pose(21.500, 35.500), new Pose(59.900, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                //舍了舍了

                // --- Parking ---
                .addPath(
                        // Path 12
                        new BezierLine(new Pose(59.760, 83.760), new Pose(28.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(270))
                //没了

                .build();
    }

    @Override
    public void init() {
        opmodeTimer = new Timer();

        // 初始化 Follower
        follower = Constants.createFollower(hardwareMap);

        // 设置机器人初始位置
        follower.setStartingPose(startPose);

        // 构建路径
        buildPaths();

        telemetry.addData("Status", "Close3CO Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        // 开始跟随主路径，true表示结束后保持位置
        follower.followPath(mainPath, true);
    }

    @Override
    public void loop() {
        // 核心更新方法
        follower.update();

        // 调试信息
        telemetry.addData("Path State", "Running");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        if (follower.isBusy()) {
            telemetry.addData("Status", "Busy");
        } else {
            telemetry.addData("Status", "Idle / Holding");
        }

        telemetry.update();
    }
}