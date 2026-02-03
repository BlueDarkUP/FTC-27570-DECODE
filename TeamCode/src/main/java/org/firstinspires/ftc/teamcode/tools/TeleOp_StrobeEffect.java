package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.LedModuleDriver;

@TeleOp(name = "Effect: Police Strobe", group = "LED")
public class TeleOp_StrobeEffect extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. 获取硬件实例
        LedModuleDriver led = hardwareMap.get(LedModuleDriver.class, "leddd");

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // 2. 初始化分配 ID
        led.assignId('A');
        sleep(200);

        telemetry.addData("Status", "Ready - Strobe Mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 3. 执行翻译后的命令：红色(FF0000) 到 蓝色(0000FF)，100%速度，100亮度
            led.setStrobeMode('A', 0xFF0000, 0x0000FF, 60, 60);

            // 保持程序运行
            while (opModeIsActive()) {
                telemetry.addData("Effect", "Running Police Strobe");
                telemetry.update();
                sleep(500);
            }
        }

        // 4. 程序关闭时自动关闭所有灯
        led.turnOff('A');
    }
}