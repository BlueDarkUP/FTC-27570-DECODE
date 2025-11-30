package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.vision.RobotVision;

@Autonomous(name = "Final Ball Sorter", group = "Competition")
public class BallSorterOpMode extends LinearOpMode {

    private String selectedInput = "GGG";
    private String selectedTarget = "GPP";

    // 目标数组用于循环切换
    private final String[] targetOptions = {"GPP", "PGP", "PPG"};
    private int targetIndex = 0;

    @Override
    public void runOpMode() {
        // 1. 初始化视觉
        RobotVision vision = new RobotVision();
        vision.init(hardwareMap); // 记得确保你的RobotVision类适配了HardwareMap

        // 2. 初始化子系统
        // 传入 this::opModeIsActive 以便子系统在死循环中检查停止信号
        SortingSubsystem sorter = new SortingSubsystem(hardwareMap, telemetry, vision, this::opModeIsActive);

        // 3. UI 设置循环 (Init Loop)
        // 对应 D-Pad 映射
        while (!isStarted() && !isStopRequested()) {
            // --- 输入选择 (D-Pad & Buttons) ---
            if (gamepad1.dpad_down)  selectedInput = "GGG";
            if (gamepad1.dpad_up)    selectedInput = "GGP";
            if (gamepad1.dpad_left)  selectedInput = "GPG";
            if (gamepad1.dpad_right) selectedInput = "GPP";
            if (gamepad1.a)          selectedInput = "PGG";
            if (gamepad1.b)          selectedInput = "PGP";
            if (gamepad1.x)          selectedInput = "PPG";
            if (gamepad1.y)          selectedInput = "PPP";

            // --- 期望切换 (Right Bumper) ---
            if (gamepad1.right_bumper) {
                targetIndex = (targetIndex + 1) % targetOptions.length;
                selectedTarget = targetOptions[targetIndex];
                // 简单的防抖动延时
                sleep(250);
            }

            // --- 显示当前状态 ---
            telemetry.addData("Config Mode", "Select Pattern");
            telemetry.addLine("-----------------");
            telemetry.addData("INPUT (D-Pad/Btn)", selectedInput);
            telemetry.addData("TARGET (RBumper)", selectedTarget);
            telemetry.addLine("-----------------");
            telemetry.addData("Vision", "AprilTag Mode Active");
            telemetry.update();
        }

        // 4. 比赛开始
        if (opModeIsActive()) {
            // 确保进入 Color 模式准备分拣
            vision.switchToColorMode();

            telemetry.addData("Status", "Running Strategy...");
            telemetry.update();

            // 执行核心逻辑
            sorter.executeStrategy(selectedInput, selectedTarget);

            // 任务结束
            telemetry.addData("Status", "Complete");
            telemetry.update();
            sleep(2000);
        }

        // 关闭视觉资源
        vision.close();
    }
}