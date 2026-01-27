package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Custom Driver for STM32 Based LED/Buzzer Module
 * I2C Address: 0x60
 * Protocol: ASCII String Commands terminated by '&'
 */
@I2cDeviceType
@DeviceProperties(
        name = "Custom LED Module (STM32)",
        xmlTag = "CustomLedModule",
        description = "Driver for daisy-chainable LED & Buzzer module"
)
public class LedModuleDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // I2C 地址 0x60 (7-bit)
    public static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x60);
    private final String TAG = "LedModule";

    public LedModuleDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false); // 注册连接状态回调
        this.deviceClient.engage(); // 激活I2C设备
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Custom LED Module";
    }

    @Override
    protected synchronized boolean doInitialize() {
        // 由于该设备没有"WHO_AM_I"寄存器，我们尝试发送一个空命令或Ping
        // 这里我们不做强制检查，直接返回true，依靠后续命令验证
        return true;
    }

    // =========================================================================
    // 核心通信方法
    // =========================================================================

    /**
     * 发送 ASCII 命令字符串到 I2C 设备
     * 自动添加 '&' 结束符
     */
    private void sendCommand(String command) {
        // 构造完整命令：命令 + "&"
        String fullCommand = command + "&";

        // 转换为 ASCII 字节数组
        byte[] payload = fullCommand.getBytes(StandardCharsets.US_ASCII);

        try {
            // 发送数据
            // 注意：STM32源码中每个字节有200us延时，REV Hub会自动处理I2C Clock Stretching
            // 如果设备反应不过来，可能需要拆分发送（但会严重影响FTC循环时间）
            this.deviceClient.write(payload);
        } catch (Exception e) {
            RobotLog.ee(TAG, "Failed to send command: " + fullCommand);
        }
    }

    /**
     * 辅助方法：将整数颜色转换为16进制字符串 (例如 0xFF0000 -> "FF0000")
     * 确保总是6位长度
     */
    private String colorToHex(int color) {
        // 屏蔽高位，只取低24位 (RGB)，格式化为大写16进制，前补0
        return String.format(Locale.US, "%06X", (color & 0xFFFFFF));
    }

    // =========================================================================
    // 用户 API (根据文档实现)
    // =========================================================================

    /**
     * 1. 分配编号 (必须在初始化时调用)
     * 命令: ASSIGN A &
     * @param id 分配给模块的编号 (例如 'A', 'B')
     */
    public void assignId(char id) {
        sendCommand("ASSIGN " + id);
    }

    /**
     * 2. 全亮模式 (Solid Color)
     * 命令: A ENT FF0000 100 &
     * @param id 目标模块编号
     * @param colorHex 颜色值 (如 0xFF0000)
     * @param brightness 亮度 (0-100)
     */
    public void setSolidColor(char id, int colorHex, int brightness) {
        String cmd = String.format(Locale.US, "%c ENT %s %d",
                id, colorToHex(colorHex), brightness);
        sendCommand(cmd);
    }

    /**
     * 3. 流水灯模式 (Wave Mode)
     * 命令: A ENTW FF00FF 00FFFF 100 &
     * @param id 目标模块编号
     * @param color1 起始颜色
     * @param color2 结束颜色
     * @param brightness 亮度 (0-100)
     */
    public void setWaveMode(char id, int color1, int color2, int brightness) {
        String cmd = String.format(Locale.US, "%c ENTW %s %s %d",
                id, colorToHex(color1), colorToHex(color2), brightness);
        sendCommand(cmd);
    }

    /**
     * 4. 单独LED控制模式
     * 命令: A ENAT L1 FF00FF L2 0000FF ... 100 &
     * 为了简化API，这里提供设置单个LED的方法，如果需要批量设置，建议在OpMode中拼接
     * 注意：文档提到"如果需要覆写原数据，写入 Lx 000000"
     */
    public void setIndividualLed(char id, int ledIndex, int colorHex, int brightness) {
        // 示例: A ENAT L1 FF0000 100
        String cmd = String.format(Locale.US, "%c ENAT L%d %s %d",
                id, ledIndex, colorToHex(colorHex), brightness);
        sendCommand(cmd);
    }

    /**
     * 5. 关闭使能 (Turn Off)
     * 命令: A ENF &
     * @param id 目标模块编号
     */
    public void turnOff(char id) {
        sendCommand(id + " ENF");
    }

    /**
     * 6. 蜂鸣器控制
     * 命令: A BENT 2700 100 &
     * @param id 目标模块编号
     * @param frequencyHz 频率 (推荐 2700)
     * @param durationMs 持续时间 (毫秒)
     */
    public void beep(char id, int frequencyHz, int durationMs) {
        String cmd = String.format(Locale.US, "%c BENT %d %d",
                id, frequencyHz, durationMs);
        sendCommand(cmd);
    }

    /**
     * 停止蜂鸣器
     * 命令: A BENT 0 0 &
     */
    public void stopBeep(char id) {
        beep(id, 0, 0);
    }
}