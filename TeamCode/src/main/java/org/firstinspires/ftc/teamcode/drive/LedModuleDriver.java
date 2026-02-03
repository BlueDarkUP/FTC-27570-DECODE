/*   MIT License
 *   Copyright (c) [2025] [Your Team Name / Team Number]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

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
 * <h1>Custom STM32 LED/Buzzer Module Driver (Firmware V2.0)</h1>
 * <h2>自定义 STM32 LED/蜂鸣器模块驱动 (固件 V2.0)</h2>
 * <p>
 * This driver provides advanced control for the STM32-based LED and Buzzer module via I2C.
 * 该驱动通过 I2C 实现对基于 STM32 的 LED 和蜂鸣器模块的高级控制。
 * </p>
 *
 * <h3>Common Color Hex Codes | 常用颜色十六进制表:</h3>
 * <table border="1">
 *   <tr><th>Color 颜色</th><th>Hex Value 十六进制值</th><th>Color 颜色</th><th>Hex Value 十六进制值</th></tr>
 *   <tr><td>Red 红色</td><td>0xFF0000</td><td>Orange 橙色</td><td>0xFFA500</td></tr>
 *   <tr><td>Green 绿色</td><td>0x00FF00</td><td>Yellow 黄色</td><td>0xFFFF00</td></tr>
 *   <tr><td>Blue 蓝色</td><td>0x0000FF</td><td>Purple 紫色</td><td>0x800080</td></tr>
 *   <tr><td>Cyan 青色</td><td>0x00FFFF</td><td>Pink 粉色</td><td>0xFFC0CB</td></tr>
 *   <tr><td>White 白色</td><td>0xFFFFFF</td><td>Gold 金色</td><td>0xFFD700</td></tr>
 *   <tr><td>Magenta 品红</td><td>0xFF00FF</td><td>Off 熄灭</td><td>0x000000</td></tr>
 * </table>
 *
 * <h3>Usage Guidelines | 使用指南:</h3>
 * <ul>
 *   <li><b>Brightness (亮度):</b> 0 - 255. Recommended: 100-200.</li>
 *   <li><b>Speed (速度):</b> Percentage based. 100 is baseline.</li>
 *   <li><b>I2C Address (地址):</b> Default is 0x60.</li>
 * </ul>
 */
@I2cDeviceType
@DeviceProperties(
        name = "Custom LED Module (STM32) V2",
        xmlTag = "CustomLedModule",
        description = "Driver for LED & Buzzer module supporting Speed% and Strobe mode"
)
public class LedModuleDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    /** Default I2C address for the module / 模块默认 I2C 地址 */
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

    /**
     * Sends a raw string command to the I2C device.
     * 向 I2C 设备发送原始字符串命令。
     *
     * @param command The string command without the '&' terminator.
     */
    private void sendCommand(String command) {
        String fullCommand = command + "&";
        byte[] payload = fullCommand.getBytes(StandardCharsets.US_ASCII);
        try {
            this.deviceClient.write(payload);
        } catch (Exception e) {
            RobotLog.ee(TAG, "I2C Write Failed: " + fullCommand);
        }
    }

    /**
     * Converts a 24-bit color integer to a hex string.
     * 将 24 位颜色整数转换为十六进制字符串。
     */
    private String colorToHex(int color) {
        return String.format(Locale.US, "%06X", (color & 0xFFFFFF));
    }

    /**
     * Assigns an identification ID to the module.
     * 为模块分配标识 ID。
     * <p>
     * After power-on, the module usually flashes orange. Assigning an ID puts it into standby.
     * 上电后模块通常呈橙色闪烁，分配 ID 后将进入待机模式（灭灯）。
     * </p>
     * @param id Character ID (e.g., 'A', 'B') / 字符 ID (如 'A', 'B')
     */
    public void assignId(char id) {
        sendCommand("ASSIGN " + id);
    }

    /**
     * Sets the module to Solid Color mode (ENT).
     * 设置为常亮模式 (ENT)。
     *
     * @param id Module ID / 模块 ID
     * @param colorHex 24-bit RGB color (e.g., 0xFF0000) / 24位 RGB 颜色
     * @param brightness Level 0-255 / 亮度等级 0-255
     */
    public void setSolidColor(char id, int colorHex, int brightness) {
        String cmd = String.format(Locale.US, "%c ENT %s %d", id, colorToHex(colorHex), brightness);
        sendCommand(cmd);
    }

    /**
     * Sets the module to Dual-Color Wave pattern (ENTW).
     * 设置为双色流水灯模式 (ENTW)。
     *
     * @param id Module ID / 模块 ID
     * @param color1 Primary RGB color / 起始颜色
     * @param color2 Background RGB color / 背景颜色
     * @param speedPercent Movement speed (100 is default) / 速度百分比 (100为基准)
     * @param brightness Level 0-255 / 亮度等级 0-255
     */
    public void setWaveMode(char id, int color1, int color2, int speedPercent, int brightness) {
        String cmd = String.format(Locale.US, "%c ENTW %s %s %d%% %d",
                id, colorToHex(color1), colorToHex(color2), speedPercent, brightness);
        sendCommand(cmd);
    }

    /**
     * Sets the module to Police Strobe mode (ENTST).
     * 设置为警灯爆闪模式 (ENTST)。
     * <p>
     * LEDs alternate between two colors in groups.
     * 灯珠分左右两组交替闪烁。
     * </p>
     * @param id Module ID / 模块 ID
     * @param color1 First flash color / 第一组颜色
     * @param color2 Second flash color / 第二组颜色
     * @param speedPercent Strobe frequency / 闪烁频率百分比
     * @param brightness Level 0-255 / 亮度等级 0-255
     */
    public void setStrobeMode(char id, int color1, int color2, int speedPercent, int brightness) {
        String cmd = String.format(Locale.US, "%c ENTST %s %s %d%% %d",
                id, colorToHex(color1), colorToHex(color2), speedPercent, brightness);
        sendCommand(cmd);
    }

    /**
     * Controls an individual LED at a specific index (ENAT).
     * 点亮单个特定位置的灯珠 (ENAT)。
     *
     * @param id Module ID / 模块 ID
     * @param ledIndex LED index starting from 0 / 灯珠索引 (从0开始)
     * @param colorHex RGB color / 颜色
     * @param brightness Level 0-255 / 亮度等级 0-255
     */
    public void setIndividualLed(char id, int ledIndex, int colorHex, int brightness) {
        String cmd = String.format(Locale.US, "%c ENAT L%d %s %d",
                id, ledIndex, colorToHex(colorHex), brightness);
        sendCommand(cmd);
    }

    /**
     * Triggers the module's buzzer (BENT).
     * 触发模块蜂鸣器鸣叫 (BENT)。
     * <p>
     * This can run simultaneously with lighting commands.
     * 该指令可与灯光指令并存。
     * </p>
     * @param id Module ID / 模块 ID
     * @param frequencyHz Frequency in Hz / 频率 (Hz)
     * @param durationMs Duration in ms / 持续时间 (毫秒)
     */
    public void beep(char id, int frequencyHz, int durationMs) {
        String cmd = String.format(Locale.US, "%c BENT %d %d", id, frequencyHz, durationMs);
        sendCommand(cmd);
    }

    /**
     * Immediately turns off all LEDs for a module (ENF).
     * 立即关闭指定模块的所有灯光 (ENF)。
     *
     * @param id Module ID / 模块 ID
     */
    public void turnOff(char id) {
        sendCommand(id + " ENF");
    }
}