package org.firstinspires.ftc.teamcode.drivers; // 建议先在TeamCode中测试，成熟后再移入SDK包路径

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

/**
 * Driver for DFRobot HuskyLens V2 Vision Sensor.
 * Based on the Python ProtocolV2 logic.
 */
@I2cDeviceType
@DeviceProperties(name = "HuskyLens V2", description = "DFRobot HuskyLens V2 AI Camera", xmlTag = "HuskyLensV2")
public class HuskyLensV2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private final String TAG = "HuskyLensV2";

    // =========================================================================
    // Protocol Constants (Ported from Python)
    // =========================================================================

    // Indexes (Python: HEADER_0_INDEX, etc.)
    private static final int HEADER_0_INDEX = 0;
    private static final int HEADER_1_INDEX = 1;
    private static final int COMMAND_INDEX = 2;
    private static final int ALGO_INDEX = 3;
    private static final int CONTENT_SIZE_INDEX = 4;
    private static final int CONTENT_INDEX = 5;
    private static final int PROTOCOL_SIZE = 6; // Header(2) + Cmd(1) + Algo(1) + Size(1) + Checksum(1)

    // Command Words
    private static final byte COMMAND_KNOCK = 0x20;
    private static final byte COMMAND_RETURN_OK = 0x40;
    private static final byte COMMAND_RETURN_ERROR = 0x41;
    // ... 我们先只用这两个来测试握手，后续再添加其他的

    // Algorithm Types
    public static final byte ALGORITHM_ANY = 0;
    public static final byte ALGORITHM_FACE_RECOGNITION = 1;
    // ... 后续添加

    // Magic Numbers
    private static final byte HEADER_0 = 0x55;
    private static final byte HEADER_1 = (byte) 0xAA;

    // Default I2C Address (0x50 in Python, 0x32 in FTC SDK usually expects 7-bit or 8-bit representation)
    // Python 0x50 usually means 7-bit address 0x50? Let's assume 0x32 (0x64 8-bit) first based on V1,
    // BUT Python code says `self.i2c_addr = 0x50`. If Python smbus uses 7-bit, it's 0x50.
    // Let's try 0x32 (default V1) first, if fails, try 0x50.
    // DFRobot Wiki says V1 is 0x32. V2 might be different. Let's make it configurable.
    public static final I2cAddr ADDRESS_V1_DEFAULT = I2cAddr.create7bit(0x32);
    public static final I2cAddr ADDRESS_V2_DEFAULT = I2cAddr.create7bit(0x50); // Python default

    // =========================================================================
    // Constructor & Initialization
    // =========================================================================

    public HuskyLensV2(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        // 尝试设置地址，如果不确定，可以在 OpMode 中 setI2cAddress 重写
        this.deviceClient.setI2cAddress(ADDRESS_V2_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        return knock();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.DFRobot;
    }

    @Override
    public String getDeviceName() {
        return "DFRobot HuskyLens V2";
    }

    // =========================================================================
    // Core Protocol Logic (The Python Port)
    // =========================================================================

    /**
     * Python: checksum(cmd)
     */
    private byte calculateChecksum(byte[] data, int length) {
        int cs = 0;
        for (int i = 0; i < length; i++) {
            cs += (data[i] & 0xFF);
        }
        return (byte) (cs & 0xFF);
    }

    /**
     * Emulates Python: husky_lens_protocol_write_begin + write fields + write_end
     * Constructs the full packet to send.
     */
    private byte[] buildCommandPacket(byte algo, byte command, byte[] content) {
        int contentLen = (content != null) ? content.length : 0;
        int totalLen = PROTOCOL_SIZE + contentLen;

        byte[] buffer = new byte[totalLen];

        buffer[HEADER_0_INDEX] = HEADER_0;
        buffer[HEADER_1_INDEX] = HEADER_1;
        buffer[COMMAND_INDEX] = command;
        buffer[ALGO_INDEX] = algo;
        buffer[CONTENT_SIZE_INDEX] = (byte) contentLen;

        if (contentLen > 0) {
            System.arraycopy(content, 0, buffer, CONTENT_INDEX, contentLen);
        }

        // Python logic: Checksum includes everything up to checksum byte
        // "for i in range(0, self.send_index): cs = cs + ..."
        buffer[totalLen - 1] = calculateChecksum(buffer, totalLen - 1);

        return buffer;
    }

    // =========================================================================
    // High Level Actions
    // =========================================================================

    /**
     * 对应 Python 的 knock()
     * Python 逻辑:
     * write_begin(ANY, KNOCK)
     * write_uint8(1)
     * write_uint8(0)
     * write_int16(0) * 4
     * write_end
     */
    public boolean knock() {
        RobotLog.ii(TAG, "Attempting KNOCK...");

        // 1. 构建 Content 数据
        // Python code writes: 1 (u8), 0 (u8), 0(i16), 0(i16), 0(i16), 0(i16)
        // Total content length: 1 + 1 + 2 + 2 + 2 + 2 = 10 bytes
        ByteBuffer contentBuf = ByteBuffer.allocate(10).order(ByteOrder.LITTLE_ENDIAN);
        contentBuf.put((byte) 1); // 可能是协议版本?
        contentBuf.put((byte) 0);
        contentBuf.putShort((short) 0);
        contentBuf.putShort((short) 0);
        contentBuf.putShort((short) 0);
        contentBuf.putShort((short) 0);

        // 2. 构建完整包
        byte[] packet = buildCommandPacket(ALGORITHM_ANY, COMMAND_KNOCK, contentBuf.array());

        // 3. 发送
        // FTC SDK 的 write 是阻塞的
        this.deviceClient.write(packet);

        // 4. 等待并读取响应
        // Python 代码中是 wait(COMMAND_RETURN_OK)。
        // 响应通常应该很快。我们先尝试读取一个固定长度的头。
        // COMMAND_RETURN_OK 的响应应该是 Header(2) + Cmd(1) + Algo(1) + Len(1) + Content + Checksum(1)
        // 通常 OK 的响应没有任何 content，或者只有很少的信息。
        // 让我们先读 6 个字节 (Header + Meta) 看看 Length 是多少。

        try {
            // 给传感器一点处理时间
            Thread.sleep(50);

            // 注意：Expansion Hub 可能会多读一个字节或吞掉一个字节。
            // 暂时先按标准读，如果不行，我们需要参考一代驱动里的 readBlocksResponse 逻辑。

            // 尝试读取头部 (5 bytes: 55 AA Cmd Algo Len)
            byte[] header = this.deviceClient.read(5);

            if (header.length < 5 || header[0] != HEADER_0 || header[1] != HEADER_1) {
                RobotLog.ee(TAG, "Knock Failed: Invalid Header: " + Arrays.toString(header));
                return false;
            }

            byte returnCmd = header[COMMAND_INDEX];
            byte returnAlgo = header[ALGO_INDEX];
            int dataLen = header[CONTENT_SIZE_INDEX] & 0xFF; // 转无符号

            RobotLog.ii(TAG, String.format("Knock Response Header: Cmd=0x%02X, Len=%d", returnCmd, dataLen));

            // 读取剩余的数据 (Data + Checksum)
            if (dataLen > 0 || true) { // 即使 dataLen 是 0，也还有一个 checksum 字节
                byte[] rest = this.deviceClient.read(dataLen + 1);
                // 这里其实应该一次性把所有读完比较好，分两次读在 I2C 上有风险（特别是 Hub bug）
                // 但先测试基本连通性。

                // 验证 Checksum (稍微复杂点，因为分段读取了，这里简化验证命令字)
            }

            if (returnCmd == COMMAND_RETURN_OK) {
                RobotLog.ii(TAG, "HuskyLens V2 Knock Successful!");
                return true;
            } else {
                RobotLog.ee(TAG, "Knock returned unexpected command: " + String.format("0x%02X", returnCmd));
                return false;
            }

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
    }
}