package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsFar;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Simple Direction Control", group = "Pedro Pathing")
public class ttt extends OpMode {

    private Follower follower;

    private double targetHeading = 0;
    private boolean isAngleActive = false;
    private boolean previousRightBumper = false;

    private final double TURN_P = 0.6;

    @Override
    public void init() {
        // 初始化 Follower
        follower = ConstantsFar.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void start() {
        // 开启 TeleOp 驱动模式，false 表示不强制使用默认的驱动逻辑，我们将手动传入参数
        follower.startTeleopDrive(false);
    }

    @Override
    public void loop() {
        /* ------------------------------------
         * 1. 逻辑控制部分
         * ------------------------------------ */

        // Left Bumper: 归零 IMU
        if (gamepad1.left_bumper) {
            // 保持当前的 X 和 Y，但将 Heading 强制设为 0
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
            targetHeading = 0; // 重置目标也为0
            isAngleActive = false;
        }

        // Right Bumper: 切换 0度 和 -114度
        if (gamepad1.right_bumper && !previousRightBumper) {
            isAngleActive = !isAngleActive; // 切换状态

            if (isAngleActive) {
                // 切换到 -114 度 (转为弧度)
                targetHeading = Math.toRadians(-114);
            } else {
                // 切换到 0 度
                targetHeading = 0;
            }
        }
        previousRightBumper = gamepad1.right_bumper;

        /* ------------------------------------
         * 2. 驱动计算部分
         * ------------------------------------ */

        double driveX = -gamepad1.left_stick_y; // 前后
        double driveY = -gamepad1.left_stick_x; // 左右
        double turnPower;

        // 如果手动推动右摇杆，优先使用手动旋转
        if (Math.abs(gamepad1.right_stick_x) > 0.05) {
            turnPower = -gamepad1.right_stick_x;
            // 更新目标朝向为当前朝向，防止松手后回弹
            targetHeading = follower.getPose().getHeading();
        } else {
            // 如果没动右摇杆，使用 P 控制器自动锁定目标角度
            double currentHeading = follower.getPose().getHeading();

            // 计算角度差 (确保走最短路径)
            double error = getAngleDifference(targetHeading, currentHeading);

            // 计算旋转力度
            turnPower = error * TURN_P;

            // 限制最大旋转速度（可选）v
            turnPower = Math.max(-1, Math.min(1, turnPower));
        }

        /* ------------------------------------
         * 3. 发送指令给 Follower
         * ------------------------------------ */

        // 参数: x速度, y速度, 旋转速度, 是否为场地坐标系(true)
        follower.setTeleOpDrive(driveX, driveY, turnPower, false);

        // 必须调用 update 来刷新里程计
        follower.update();

        // Telemetry 显示调试信息
        telemetry.addData("Target Angle (Deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Current Angle (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Mode", isAngleActive ? "-114 Degrees" : "0 Degrees");
        telemetry.update();
    }

    /**
     * 辅助方法：计算两个角度之间的最小差值 (-PI 到 PI)
     */
    private double getAngleDifference(double target, double current) {
        double difference = target - current;
        while (difference < -Math.PI) difference += 2 * Math.PI;
        while (difference > Math.PI) difference -= 2 * Math.PI;
        return difference;
    }
}