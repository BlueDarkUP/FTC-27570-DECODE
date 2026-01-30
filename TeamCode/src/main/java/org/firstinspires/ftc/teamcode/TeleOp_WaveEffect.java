package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.LedModuleDriver;

@TeleOp(name = "Effect: Purple Blue Wave", group = "LED")
public class TeleOp_WaveEffect extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. 获取硬件实例 (需在配置中命名为 "ledModule")
        LedModuleDriver led = hardwareMap.get(LedModuleDriver.class, "leddd");

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // 2. 初始化分配 ID
        led.assignId('A');
        sleep(200); // 给硬件一点处理时间

        telemetry.addData("Status", "Ready - Wave Mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 3. 执行翻译后的命令：紫色(FF00FF) 到 蓝色(0000FF)，100%速度，100亮度
            led.setWaveMode('A', 0xFF00FF, 0x00FFFF, 200, 60);

            // 保持程序运行
            while (opModeIsActive()) {
                telemetry.addData("Effect", "Running Purple Blue Wave");
                telemetry.update();
                sleep(500);
            }
        }

        // 4. 程序关闭时自动关闭所有灯
        led.turnOff('A');
    }
}