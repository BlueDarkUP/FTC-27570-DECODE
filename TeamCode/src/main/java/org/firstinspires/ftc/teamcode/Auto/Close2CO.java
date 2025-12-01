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

@Autonomous(name = "Close2CO", group = "PedroPathing")
public class Close2CO extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private PathChain mainPath;

    // 该死的起始点位，车辆右下角紧贴蓝方球门小角
    private final Pose startPose = new Pose(33.600, 135.560, Math.toRadians(270));

    public void buildPaths() {
        mainPath = follower.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(33.600, 135.560), new Pose(48.500, 95.900))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-48))
                // 这段路径完成后发射预制

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
                        new BezierLine(new Pose(20.000, 59.640), new Pose(59.9, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                //舍了舍了

                .addPath(
                        // Path 9
                        new BezierLine(new Pose(59.880, 84.000), new Pose(28.000, 70.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(270))

                //没了

                .build();
    }

    /**
     * 初始化阶段：这是按下 INIT 按钮时运行的代码
     */
    @Override
    public void init() {
        // 初始化计时器
        opmodeTimer = new Timer();

        // 1. 创建 Follower 实例 (引用 Constants 类)
        follower = Constants.createFollower(hardwareMap);

        // 2. 设置起始姿态 (告诉机器人它现在在哪里)
        follower.setStartingPose(startPose);

        // 3. 构建所有路径
        buildPaths();

        // 反馈信息
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * 开始阶段：这是按下 START (Play) 按钮时运行的代码
     */
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        // 开始跟随路径
        // true 参数表示保持在路径终点（Hold Position）
        follower.followPath(mainPath, true);
    }

    /**
     * 循环阶段：这是程序运行时不断循环的代码
     */
    @Override
    public void loop() {
        // 非常重要：必须不断调用 update() 来计算驱动功率
        follower.update();

        // Telemetry 数据，用于 Driver Hub 调试
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        // 显示当前是否正在忙于跑路径
        if (follower.isBusy()) {
            telemetry.addData("Status", "Running Path...");
        } else {
            telemetry.addData("Status", "Path Completed (Holding)");
        }

        telemetry.update();
    }
}