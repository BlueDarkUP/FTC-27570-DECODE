package org.firstinspires.ftc.teamcode.vision.QuickScope;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "AprilTag Test (Pose & ID)", group = "Test")
public class ApriltagTest extends LinearOpMode {

    private AprilTagLocalizer localizer;

    @Override
    public void runOpMode() {
        // 1. 初始化 Localizer
        // 请确保 Robot Controller 的配置中有一个名为 "Webcam 1" 的摄像头
        telemetry.addLine("正在初始化摄像头...");
        telemetry.update();

        try {
            localizer = new AprilTagLocalizer(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("初始化失败", e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        telemetry.addData("状态", "已准备就绪");
        telemetry.addData("说明", "按下 Play 开始测试");
        telemetry.update();

        waitForStart();

        // 2. 主循环
        while (opModeIsActive()) {

            // --- 功能 A: 检测 Spike Mark (ID 21, 22, 23) ---
            // 这部分用于判断物体位置，返回 1, 2, 3 (如果没有则是 0)
            int spikeLocation = localizer.getSpikeMarkLocation();

            // --- 功能 B: 获取机器人全场定位 ---
            // 这部分通过场地标签计算坐标。如果只看到 ID 21-23，这里可能会返回 null (取决于库配置)
            Pose2D robotPose = localizer.getRobotPose();

            // 3. 显示数据到 Driver Station
            telemetry.addData("=== Spike Mark 检测 (ID 21-23) ===", "");
            if (spikeLocation == 0) {
                telemetry.addData("检测结果", "未检测到目标 (0)");
            } else {
                String positionStr = "";
                switch (spikeLocation) {
                    case 1: positionStr = "左 (Left - ID 21)"; break;
                    case 2: positionStr = "中 (Center - ID 22)"; break;
                    case 3: positionStr = "右 (Right - ID 23)"; break;
                }
                telemetry.addData("检测结果", "%d -> %s", spikeLocation, positionStr);
            }

            telemetry.addLine(); // 空行分隔

            telemetry.addData("=== 机器人定位 (Robot Pose) ===", "");
            if (robotPose != null) {
                // 将单位转换为 CM 和度数显示，方便阅读
                telemetry.addData("X 坐标", "%.2f cm", robotPose.getX(DistanceUnit.CM));
                telemetry.addData("Y 坐标", "%.2f cm", robotPose.getY(DistanceUnit.CM));
                telemetry.addData("Heading", "%.2f deg", robotPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addData("定位状态", "丢失定位 (无有效定位标签)");
                telemetry.addLine("注: ID 21-23 可能不包含场地坐标信息");
            }

            telemetry.update();

            // 稍微休眠一下避免 CPU 过载，视觉处理通常不需要超高频率刷新
            sleep(20);
        }

        // 4. 程序结束时关闭资源
        localizer.close();
    }
}