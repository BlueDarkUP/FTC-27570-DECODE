package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.LedModuleDriver;

/**
 * LED 模块功能演示 TeleOp
 * LED Module Feature Demonstration TeleOp
 *
 * 此程序展示了如何通过手柄按键触发 LED 的各种模式和蜂鸣器。
 * This program demonstrates how to trigger various LED modes and the buzzer using gamepad buttons.
 */
@TeleOp(name = "LED Module Demo (V2)", group = "Demo")
public class LedDemoOpMode extends LinearOpMode {

    private LedModuleDriver led;
    private final char MODULE_ID = 'A'; // 默认模块 ID / Default module ID

    @Override
    public void runOpMode() {
        // 1. 初始化驱动 / Initialize the driver
        // 请确保在 Robot Configuration 中将设备命名为 "CustomLedModule"
        // Ensure the device is named "CustomLedModule" in the Robot Configuration
        led = hardwareMap.get(LedModuleDriver.class, "CustomLedModule");

        telemetry.addLine(">> 已连接 LED 模块 / LED Module Connected");
        telemetry.addLine(">> 按下 START 开始演示 / Press START to begin");
        telemetry.update();

        waitForStart();

        // 2. 分配 ID / Assign ID
        // 模块上电时通常闪烁橙色，分配 ID 后会停止闪烁进入待机
        // Module usually flashes orange on power-up; assigning an ID stops the flashing and enters standby
        led.assignId(MODULE_ID);
        sleep(200); // 等待命令处理 / Wait for command processing

        while (opModeIsActive()) {

            // --- 常亮模式展示 / Solid Color Mode ---
            if (gamepad1.a) {
                // 设置为红色，亮度 200 / Set to Red, Brightness 200
                led.setSolidColor(MODULE_ID, 0xFF0000, 200);
                telemetry.addData("Mode", "Solid Red");
            }

            if (gamepad1.b) {
                // 设置为绿色，亮度 200 / Set to Green, Brightness 200
                led.setSolidColor(MODULE_ID, 0x00FF00, 200);
                telemetry.addData("Mode", "Solid Green");
            }

            // --- 流水模式展示 / Wave Mode ---
            if (gamepad1.x) {
                // 蓝色与青色交替流水，速度 150%，亮度 255
                // Blue and Cyan wave, speed 150%, brightness 255
                led.setWaveMode(MODULE_ID, 0x0000FF, 0x00FFFF, 150, 255);
                telemetry.addData("Mode", "Ocean Wave");
            }

            // --- 爆闪模式展示 / Strobe Mode ---
            if (gamepad1.y) {
                // 红蓝警灯爆闪，速度 250% (极快)，亮度 255
                // Red and Blue "Police" strobe, speed 250% (very fast), brightness 255
                led.setStrobeMode(MODULE_ID, 0xFF0000, 0x0000FF, 250, 255);
                telemetry.addData("Mode", "Police Strobe");
            }

            // --- 单个灯珠控制展示 / Individual LED Control ---
            if (gamepad1.dpad_up) {
                // 仅点亮第 0 号灯珠为白色 / Turn on only LED index 0 to White
                led.setIndividualLed(MODULE_ID, 0, 0xFFFFFF, 255);
                telemetry.addData("Mode", "Individual LED 0");
            }

            // --- 蜂鸣器控制展示 / Buzzer Control ---
            if (gamepad1.left_bumper) {
                // 发出 1000Hz 的声音，持续 100ms / 1000Hz beep for 100ms
                led.beep(MODULE_ID, 1000, 100);
                telemetry.addData("Buzzer", "Short Beep");
                sleep(150); // 防止连续触发导致 I2C 堵塞 / Prevent spamming I2C
            }

            if (gamepad1.right_bumper) {
                // 发出 500Hz 的低音，持续 500ms / 500Hz low beep for 500ms
                led.beep(MODULE_ID, 500, 500);
                telemetry.addData("Buzzer", "Long Low Beep");
                sleep(550);
            }

            // --- 关闭灯光 / Turn Off ---
            if (gamepad1.dpad_down) {
                led.turnOff(MODULE_ID);
                telemetry.addData("Mode", "Off");
            }

            // 状态更新 / Telemetry Update
            telemetry.addLine("\n使用说明 / Controls:");
            telemetry.addLine("A: 红色常亮 / Solid Red");
            telemetry.addLine("B: 绿色常亮 / Solid Green");
            telemetry.addLine("X: 流水模式 / Wave Mode");
            telemetry.addLine("Y: 爆闪模式 / Strobe Mode");
            telemetry.addLine("D-Pad Up: 点亮单个灯珠 / Single LED");
            telemetry.addLine("D-Pad Down: 关闭灯光 / Turn Off");
            telemetry.addLine("L/R Bumper: 蜂鸣器测试 / Buzzer Test");
            telemetry.update();

            // 小段延迟保护 I2C 总线 / Small delay to protect I2C bus
            idle();
        }
    }
}