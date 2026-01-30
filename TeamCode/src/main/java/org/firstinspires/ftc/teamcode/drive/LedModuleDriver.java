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
 * Enhanced Driver for STM32 Based LED/Buzzer Module (Firmware V2.0 Compatible)
 * Compatible with Speed% parameters and ENTST (Police Strobe) mode.
 */
@I2cDeviceType
@DeviceProperties(
        name = "Custom LED Module (STM32) V2",
        xmlTag = "CustomLedModule",
        description = "Driver for LED & Buzzer module supporting Speed% and Strobe mode"
)
public class LedModuleDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x60);
    private final String TAG = "LedModuleV2";

    public LedModuleDriver(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Custom LED Module V2";
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    private void sendCommand(String command) {
        String fullCommand = command + "&";
        byte[] payload = fullCommand.getBytes(StandardCharsets.US_ASCII);
        try {
            this.deviceClient.write(payload);
        } catch (Exception e) {
            RobotLog.ee(TAG, "I2C Write Failed: " + fullCommand);
        }
    }

    private String colorToHex(int color) {
        return String.format(Locale.US, "%06X", (color & 0xFFFFFF));
    }

    // =========================================================================
    // 用户 API (兼容最新固件)
    // =========================================================================

    /**
     * 分配编号。上电后灯光呈橙色闪烁，分配ID后进入待机模式。
     */
    public void assignId(char id) {
        sendCommand("ASSIGN " + id);
    }

    /**
     * 全亮模式 (ENT)
     */
    public void setSolidColor(char id, int colorHex, int brightness) {
        String cmd = String.format(Locale.US, "%c ENT %s %d", id, colorToHex(colorHex), brightness);
        sendCommand(cmd);
    }

    /**
     * 流水灯模式 (ENTW) - 增强版，支持速度百分比
     * @param speedPercent 速度百分比 (1-500)，100为基准速度
     */
    public void setWaveMode(char id, int color1, int color2, int speedPercent, int brightness) {
        String cmd = String.format(Locale.US, "%c ENTW %s %s %d%% %d",
                id, colorToHex(color1), colorToHex(color2), speedPercent, brightness);
        sendCommand(cmd);
    }

    /**
     * 警灯爆闪模式 (ENTST) - [最新固件功能]
     * 灯珠分为左右两区交替闪烁。
     * @param speedPercent 闪烁快慢 (例如 200% 为极快)
     */
    public void setStrobeMode(char id, int color1, int color2, int speedPercent, int brightness) {
        String cmd = String.format(Locale.US, "%c ENTST %s %s %d%% %d",
                id, colorToHex(color1), colorToHex(color2), speedPercent, brightness);
        sendCommand(cmd);
    }

    /**
     * 独立灯珠控制 (ENAT)
     */
    public void setIndividualLed(char id, int ledIndex, int colorHex, int brightness) {
        String cmd = String.format(Locale.US, "%c ENAT L%d %s %d",
                id, ledIndex, colorToHex(colorHex), brightness);
        sendCommand(cmd);
    }

    /**
     * 蜂鸣器控制 (BENT)
     * 支持在任意状态下触发。
     */
    public void beep(char id, int frequencyHz, int durationMs) {
        String cmd = String.format(Locale.US, "%c BENT %d %d", id, frequencyHz, durationMs);
        sendCommand(cmd);
    }

    public void turnOff(char id) {
        sendCommand(id + " ENF");
    }
}