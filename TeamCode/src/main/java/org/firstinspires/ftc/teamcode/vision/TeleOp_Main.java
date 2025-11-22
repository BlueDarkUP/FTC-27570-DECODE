// 文件名: TeleOp_Main.java
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// 导入所有必需的自定义类
import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name = "TeleOp Main - Intake Focused", group = "Linear Opmode")
public class TeleOp_Main extends LinearOpMode {

    // --- 硬件设备声明 ---
    private DcMotor LeftFrontMotor, LeftBehindMotor, RightFrontMotor, RightBehindMotor;
    private DcMotor InhaleMotor, IntakeMotor;
    // 注意: ClassifyMotor 现在由 Deadeye 类内部管理，这里不再需要声明

    // --- 视觉与定位模块 ---
    private Deadeye deadeye;
    private GoBildaPinpointDriver odo;

    // --- 状态变量 ---
    private boolean isIntakeMode = false;
    private boolean rightBumperPreviouslyPressed = false;

    // --- 常量与调节参数 ---
    private static final double INTAKE_Y_GAIN = 0.03; // 辅助吸入时，前进后退的修正强度
    private static final double INTAKE_X_GAIN = 0.04; // 辅助吸入时，左右平移的修正强度

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("状态", "初始化完成，等待开始...");
        telemetry.update();

        waitForStart();

        // --- 主循环 ---
        while (opModeIsActive()) {
            // 首先更新所有传感器和模块
            odo.update();
            deadeye.update(); // 这一步至关重要，它会处理所有视觉和分类舵机逻辑

            // 模式切换 (只保留吸入模式的切换)
            handleToggles();

            // 获取手柄输入 (原始的、以机器人为中心的输入)
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_trigger - gamepad1.left_trigger;

            // --- 全局无头模式 (Field-Centric Drive) ---
            // 获取当前航向角（弧度）
            double currentHeadingRad = odo.getHeading(AngleUnit.RADIANS);
            // 根据航向角旋转手柄输入向量，得到场地坐标系下的期望运动方向
            double rotatedStrafe = strafe * Math.cos(-currentHeadingRad) - drive * Math.sin(-currentHeadingRad);
            double rotatedDrive = strafe * Math.sin(-currentHeadingRad) + drive * Math.cos(-currentHeadingRad);

            // 根据模式执行相应逻辑
            if (isIntakeMode) {
                // 在吸入模式下，应用视觉辅助
                double[] intakePowers = runIntakeModeLogic(rotatedDrive, rotatedStrafe);
                rotatedDrive = intakePowers[0];
                rotatedStrafe = intakePowers[1];
            } else {
                // 在手动模式下，关闭吸入电机
                runManualModeLogic();
            }

            // --- 底盘驱动和功率归一化 (使用旋转后的值) ---
            double leftFrontPower = rotatedDrive + rotatedStrafe + turn;
            double rightFrontPower = rotatedDrive - rotatedStrafe - turn;
            double leftBehindPower = rotatedDrive - rotatedStrafe + turn;
            double rightBehindPower = rotatedDrive + rotatedStrafe - turn;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBehindPower));
            max = Math.max(max, Math.abs(rightBehindPower));
            if (max > 1.0) {
                leftFrontPower /= max; rightFrontPower /= max;
                leftBehindPower /= max; rightBehindPower /= max;
            }

            LeftFrontMotor.setPower(leftFrontPower);
            RightFrontMotor.setPower(rightFrontPower);
            LeftBehindMotor.setPower(leftBehindPower);
            RightBehindMotor.setPower(rightBehindPower);

            // 显示遥测数据
            displayTelemetry();
        }

        // 停止OpMode时，关闭视觉
        deadeye.stop();
    }

    /**
     * 【方法已修改】处理吸入模式的逻辑。
     * 现在只负责开启吸入马达和提供驾驶辅助。分类舵机的控制完全交给 Deadeye 类。
     */
    private double[] runIntakeModeLogic(double currentDrive, double currentStrafe) {
        // 1. 开启吸入马达
        InhaleMotor.setPower(0.6);
        IntakeMotor.setPower(1.0);

        double newDrive = currentDrive;
        double newStrafe = currentStrafe;

        // 2. 寻找最适合吸入的目标，并进行驾驶辅助
        LLResultTypes.DetectorResult target = deadeye.getTargetClosestToAnchor();
        if (target != null) {
            double[] errors = deadeye.calculateAlignmentError(target);
            if (errors != null) {
                // 根据视觉误差，修正驾驶员的输入，实现自动对准
                newDrive = Range.clip(errors[1] * INTAKE_Y_GAIN, -1.0, 1.0);
                newStrafe = Range.clip(errors[0] * INTAKE_X_GAIN, -1.0, 1.0);
            }
        }

        // 3. 【重要】不再需要手动控制分类舵机。
        // deadeye.update() 方法会自动检测像素是否进入分类区，并执行完整的分类动作序列。

        return new double[]{newDrive, newStrafe};
    }

    /**
     * 处理手动驾驶模式的逻辑。
     */
    private void runManualModeLogic() {
        // 关闭吸入马达
        InhaleMotor.setPower(0);
        IntakeMotor.setPower(0);
        // Deadeye 在其 IDLE 状态会自动将分类舵机功率设为0，所以这里无需干预。
    }

    /**
     * 初始化所有硬件设备。
     */
    private void initializeHardware() {
        // --- 底盘电机 ---
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        LeftBehindMotor = hardwareMap.get(DcMotor.class, "LeftBehindMotor");
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        RightBehindMotor = hardwareMap.get(DcMotor.class, "RightBehindMotor");
        LeftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftBehindMotor.setDirection(DcMotor.Direction.FORWARD);
        RightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        RightBehindMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBehindMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBehindMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 吸入电机 ---
        InhaleMotor = hardwareMap.get(DcMotor.class, "InhaleMotor");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        InhaleMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // --- 视觉与定位模块 ---
        // Deadeye 会自动初始化 Limelight 和名为 "ClassifyMotor" 的CR舵机
        deadeye = new Deadeye(hardwareMap);
        deadeye.start();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(85.0, -180.0, org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    /**
     * 处理模式切换的逻辑。
     */
    private void handleToggles() {
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !rightBumperPreviouslyPressed) {
            isIntakeMode = !isIntakeMode;
        }
        rightBumperPreviouslyPressed = rightBumper;
    }

    /**
     * 在屏幕上显示遥测数据。
     */
    private void displayTelemetry() {
        telemetry.addData("--- 模式 ---", isIntakeMode ? "吸入模式 (视觉辅助)" : "手动驾驶 (无头模式)");

        telemetry.addData("--- 里程计 ---", "");
        Pose2D currentPose = odo.getPosition();
        telemetry.addData("位置 (X, Y, H)", String.format(Locale.US, "%.1f, %.1f, %.1f°",
                currentPose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM),
                currentPose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM),
                currentPose.getHeading(AngleUnit.DEGREES)));

        telemetry.addData("--- 智能分类 ---", "");
        telemetry.addData("正在分类中?", deadeye.isClassifying());
        telemetry.addData("当前分类计划", deadeye.getClassificationPlan().toString());
        telemetry.addData("已吸入计数", deadeye.getSuccessfulIntakeCount());

        telemetry.update();
    }
}