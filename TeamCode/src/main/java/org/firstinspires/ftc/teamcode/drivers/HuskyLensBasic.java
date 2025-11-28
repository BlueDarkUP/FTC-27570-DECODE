package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "HuskyLens (Basic)", description = "Simplified HuskyLens I2C driver skeleton", xmlTag = "HuskyLensBasic")
public class HuskyLensBasic extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private final static I2cAddr I2C_ADDR = I2cAddr.create7bit(0x50);

    public HuskyLensBasic(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2C_ADDR);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        // 简单检查: 试着 read 一个 byte，看是否有响应 (non-null / non-zero)
        try {
            byte[] data = deviceClient.read(1);
            return data != null && data.length > 0;
        } catch (Exception e) {
            return false;
        }
    }

    @Override
    public String getDeviceName() {
        return "HuskyLens Basic";
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    /** Basic ping / test – 读取 n bytes 并返回是否成功 */
    public boolean ping(int n) {
        try {
            byte[] d = deviceClient.read(n);
            return d != null && d.length == n;
        } catch (Exception e) {
            return false;
        }
    }
}
