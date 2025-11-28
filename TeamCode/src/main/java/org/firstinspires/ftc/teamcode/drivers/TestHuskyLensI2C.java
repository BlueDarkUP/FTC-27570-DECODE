package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;  // 或 @Autonomous
import org.firstinspires.ftc.teamcode.drivers.HuskyLensBasic;  // 假设 driver 在这个包
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Simple OpMode: 测试 HuskyLensBasic I2C 设备是否可 ping / 读取
 */
@TeleOp(name="Test HuskyLens I2C", group="Test")
public class TestHuskyLensI2C extends LinearOpMode {
    private HuskyLensBasic husky;

    @Override
    public void runOpMode() {
        // 通过 hardwareMap 获取 device。
        // “huskylens” 是你在 Robot Controller → Configure Robot 中设的 name。
        husky = hardwareMap.get(HuskyLensBasic.class, "huskylens");

        telemetry.addData("Status", "Initialized; waiting for start");
        telemetry.update();

        // 等待 Driver Station 按 START
        waitForStart();

        // 检查设备是否 alive、是否能读取
        if (husky != null) {
            boolean ok = husky.ping(4);   // 尝试从 I2C 设备读取 4 个字节
            telemetry.addData("HuskyLens ping(4)", ok ? "SUCCESS" : "FAIL");
        } else {
            telemetry.addData("HuskyLens", "NULL — not found");
        }
        telemetry.update();

        // 保持 opMode 运行一段时间，使你能在 Driver Station 看 telemetry
        sleep(5000);
    }
}
