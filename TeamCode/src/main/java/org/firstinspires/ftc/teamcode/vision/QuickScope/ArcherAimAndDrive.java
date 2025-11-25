package org.firstinspires.ftc.teamcode.vision.QuickScope;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.IPoseProvider;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;

import java.util.Locale;

@TeleOp(name = "Archer - 闭环定位与逻辑测试 (强修正)", group = "Tests")
public class ArcherAimAndDrive extends LinearOpMode {

    // 传感器与逻辑对象
    private IMU imu;
    private AprilTagLocalizer aprilTagLocalizer;
    private IPoseProvider pinpointPoseProvider;
    private ArcherLogic archerLogic;
    private ElapsedTime runtime = new ElapsedTime();

    // 状态变量
    private double headingOffset = 0.0;
    private String targetAlliance = "Red";

    // 调试变量：记录上一次视觉修正的时间
    private double lastVisionUpdateTime = 0.0;

    @Override
    public void runOpMode() {

        // 1. 初始化
        initializeSensorsAndLogic();

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("初始化完成");
        telemetry.addLine("模式: 全程视觉闭环 (看到Tag即强制修正坐标)");
        telemetry.update();

        waitForStart();

        // 2. 初始处理
        // 如果开始时正好看到Tag，先进行一次校准
        performVisionCorrection();

        // 注意：这里不再调用 aprilTagLocalizer.close()，让它一直跑！

        runtime.reset();

        // 3. 主循环
        while (opModeIsActive()) {

            // --- A. 闭环修正核心 (Closed Loop Correction) ---
            // 每一帧都尝试去“看”一眼
            // 如果这一帧看到了Tag，performVisionCorrection 内部会把坐标强行掰回来
            boolean visionUpdated = performVisionCorrection();

            // --- B. 传感器数据更新 ---
            // 无论是否修正，都要更新Pinpoint以获取最新的速度和微小的位移
            pinpointPoseProvider.update();

            // 获取并处理里程计数据 (此时如果是刚修正完，这里就是修正后的准确坐标)
            double robotX_cm = -pinpointPoseProvider.getX(DistanceUnit.CM);
            double robotY_cm = pinpointPoseProvider.getY(DistanceUnit.CM);

            // 归一化
            double normalizationX = robotX_cm / 365.76;
            double normalizationY = robotY_cm / 365.76;

            // 获取速度
            double cartesianVelX_m_s = -pinpointPoseProvider.getXVelocity(DistanceUnit.MM) / 1000.0;
            double cartesianVelY_m_s = pinpointPoseProvider.getYVelocity(DistanceUnit.MM) / 1000.0;
            double speed_m_s = Math.hypot(cartesianVelX_m_s, cartesianVelY_m_s);

            // 计算方向
            double direction_rad = Math.atan2(cartesianVelY_m_s, cartesianVelX_m_s);
            double direction_deg = Math.toDegrees(direction_rad);
            if (direction_deg < 0) direction_deg += 360;

            // --- C. 手柄交互 ---
            if (gamepad1.x) targetAlliance = "Blue";
            if (gamepad1.b) targetAlliance = "Red";

            if (gamepad1.a) {
                // 手动强制归零 (救急用)
                pinpointPoseProvider.reset();
                imu.resetYaw();
                headingOffset = 0;
            }

            // --- D. 逻辑计算 ---
            CalculationParams currentParams = new CalculationParams(
                    normalizationX,
                    normalizationY,
                    speed_m_s,
                    direction_deg,
                    targetAlliance
            );

            LaunchSolution solution = archerLogic.calculateSolution(currentParams);

            // --- E. 数据反馈 ---
            updateTelemetry(solution, targetAlliance, normalizationX, normalizationY, visionUpdated);
        }

        // 结束时关闭视觉
        aprilTagLocalizer.close();
    }

    // =================================================================================================
    //                                     核心方法
    // =================================================================================================

    /**
     * 视觉强修正逻辑
     * @return 如果本帧进行了修正返回 true，否则返回 false
     */
    private boolean performVisionCorrection() {
        // 1. 获取视觉计算出的绝对坐标
        Pose2D visionPose = aprilTagLocalizer.getRobotPose();

        // 2. 如果看到了 (不为null)
        if (visionPose != null) {
            // [强修正 1]：直接覆盖里程计的当前坐标
            // Pinpoint驱动会接受这个新坐标，并以此为基础继续积分
            pinpointPoseProvider.setPose(visionPose);

            // [强修正 2]：重新计算IMU的偏移量
            // 当前准确航向 (视觉) - IMU原始航向 = 新的偏移量
            // 这样 getRobotFieldHeading() 就会瞬间变准
            double visionHeading = visionPose.getHeading(AngleUnit.DEGREES);
            double rawImuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            headingOffset = visionHeading - rawImuYaw;

            // 记录时间
            lastVisionUpdateTime = runtime.seconds();
            return true;
        }
        return false;
    }

    private void initializeSensorsAndLogic() {
        imu = hardwareMap.get(IMU.class, "imu");
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        pinpointPoseProvider = new PinpointPoseProvider(hardwareMap, "odo");
        pinpointPoseProvider.initialize();
        archerLogic = new ArcherLogic();
    }

    private double getRobotFieldHeading() {
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double correctedYaw = rawYaw + headingOffset;
        return normalizeAngle(correctedYaw);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void updateTelemetry(LaunchSolution solution, String alliance, double x, double y, boolean visionUpdated) {
        telemetry.addLine("=== Archer 闭环修正监控 ===");

        // 显示定位状态
        if (visionUpdated) {
            telemetry.addData("定位源", ">> 视觉 VISION <<");
        } else {
            // 如果超过2秒没看到Tag，提示依靠里程计
            if (runtime.seconds() - lastVisionUpdateTime > 2.0) {
                telemetry.addData("定位源", "仅里程计 (Odometry Only)");
            } else {
                telemetry.addData("定位源", "里程计 (近期有视觉修正)");
            }
        }
        telemetry.addData("上次修正时间", "%.1f s 前", runtime.seconds() - lastVisionUpdateTime);

        telemetry.addLine("\n--- 机器人状态 ---");
        telemetry.addData("坐标 (Norm)", "X: %.3f, Y: %.3f", x, y);
        telemetry.addData("航向 (Field)", "%.2f deg", getRobotFieldHeading());
        telemetry.addData("IMU偏移量", "%.2f deg", headingOffset);

        telemetry.addLine("\n--- 解算结果 ---");
        if (solution != null) {
            telemetry.addData("目标", alliance);
            telemetry.addData("发射仰角", "%.2f", solution.launcherAngle);
            telemetry.addData("底盘朝向", "%.2f", solution.aimAzimuthDeg);
        } else {
            telemetry.addLine(">> 无解 (No Solution) <<");
        }

        telemetry.update();
    }
}