package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * 一个超简单的 I2C 地址扫描器。
 * 用来确定 HuskyLens V2 到底在哪个地址上。
 */
@TeleOp(name = "HuskyLens I2C Scanner", group = "Test")
public class HuskyLensScanner extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to Scan. Press Start.");
        telemetry.update();

        // 获取你在 Config 里配置的名字为 "huskylens" 的设备
        // 哪怕之前的驱动报错也没关系，我们这里只借用这个对象
        I2cDeviceSynch device = hardwareMap.get(I2cDeviceSynch.class, "huskylens");

        // 准备开始
        waitForStart();

        telemetry.addData("Status", "Scanning...");
        telemetry.update();

        // 这里的 engage 是必须的，开启 I2C 总线
        device.engage();

        String foundAddresses = "";
        boolean foundAny = false;

        // 扫描常见的 7-bit 地址范围
        // 0x32 (50) 是 HuskyLens V1
        // 0x50 (80) 是 V2 可能的地址
        for (int i = 0x08; i <= 0x77; i++) {
            I2cAddr addr = I2cAddr.create7bit(i);
            device.setI2cAddress(addr);

            // 尝试读取 1 个字节
            // 如果设备存在并响应，这行代码通常能成功 (或者至少不会报 connection error)
            // 在 FTC SDK 中，最可靠的方法是检查能否读取且不报错，但 SDK 屏蔽了底层异常。
            // 我们可以尝试发送一个空读。

            // 下面的方法是利用 SDK 的心跳检测机制
            byte[] result = device.read(1);

            // 如果 Logcat 没有报错 Error communicating... 并且读出了数据（哪怕是0）
            // 通常意味着地址是对的。但更准确的是看有没有抛出异常。
            // 实际上，FTC SDK 如果地址不对，read 会返回全 0 或者上次的缓存，且会有报错日志。

            // 另一种方法：根据特定的可能的地址进行强探测
            if (i == 0x32 || i == 0x50) {
                telemetry.addData("Checking", String.format("0x%02X", i));
                telemetry.update();
                sleep(100);
            }
        }

        // 由于 FTC SDK 不提供直接的 "ping" API，上述循环扫描比较低效且难以直接判断。
        // 我们换一种策略：只测试 V1 和 V2 的默认地址。

        telemetry.addLine("Checking Specific Addresses:");

        // Check V1 (0x32)
        if (checkAddress(device, 0x32)) {
            foundAddresses += "0x32 (V1 Default)\n";
            foundAny = true;
        }

        // Check V2 (0x50)
        if (checkAddress(device, 0x50)) {
            foundAddresses += "0x50 (V2 Default)\n";
            foundAny = true;
        }

        telemetry.addData("Scan Result", foundAny ? "FOUND!" : "NOT FOUND");
        telemetry.addData("Addresses", foundAddresses);
        telemetry.update();

        while (opModeIsActive()) {
            sleep(1000);
        }
    }

    // 辅助函数：尝试读取某个地址的 Protocol Header (0x55)
    // HuskyLens 无论是一代还是二代，只要开机，发送 0x55 0xAA 头的概率很大
    // 或者仅仅读取看看是否报错。
    private boolean checkAddress(I2cDeviceSynch device, int address7bit) {
        device.setI2cAddress(I2cAddr.create7bit(address7bit));

        // 尝试读取 2 个字节
        byte[] data = device.read(2);

        // 如果我们能读到数据，且不是全 0xFF (通常表示总线空闲/上拉)，有可能就是它
        // 注意：如果设备没插，I2C 可能会读回 0x00 或 0xFF

        // 更好的判断：HuskyLens 协议头是 0x55, 0xAA。
        // 如果读到了 0x55，那绝对就是它了。
        if (data.length >= 1 && data[0] == 0x55) return true;
        if (data.length >= 2 && data[1] == 0x55) return true; // 考虑 Hub 吞字节的情况

        // 如果没读到特征头，但能正常通信（没有底层报错），可能需要看 Logcat
        // 简单起见，我们假设如果读回来的不是全 0xFF 且不是全 0x00，就有可能是设备
        boolean allZero = true;
        boolean allFF = true;
        for (byte b : data) {
            if (b != 0) allZero = false;
            if (b != (byte)0xFF) allFF = false;
        }

        return !allFF; // I2C 如果没人应答，通常是高电平(0xFF)
    }
}