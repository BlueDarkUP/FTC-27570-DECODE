package org.firstinspires.ftc.teamcode;

// import com.pedropathing.follower.Follower; // <-- 1. 移除 PedroPathing 相关的导入
// import com.pedropathing.geometry.Pose; // <-- 2. 移除 PedroPathing 相关的导入
// import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsFar; // <-- 3. 移除 PedroPathing 相关的导入

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@TeleOp(name="TeleOpProMax_Final_Optimized_IMU", group="Linear Opmode") // 新名字以区分
public class TeleOpProMax extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;

    // 传感器
    private DigitalChannel jujuSensor = null;
    private DigitalChannel bigjuju = null;

    // 电机与舵机
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor intake = null;
    private DcMotor blender = null;

    // 双发射电机
    private DcMotorEx shooterMotor = null;
    private DcMotorEx shooterHelper = null;

    private Servo servoRP = null;
    private Servo servoLP = null;
    private CRServo holdServo = null;

    // 状态变量
    private boolean lastYState = false;
    private boolean state = false; // true = 射击模式
    private boolean intakeMode = false; // true = 进弹模式
    private boolean isManualStopped = false;

    // PID与常量
    public static final double MOTOR_TICK_COUNT = 28;
    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;

    // 射击系统PID参数
    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.006;
    private static final double SHOOTER_F = 0.0004;
    private static final double SHOOTER_D = 0.00001;

    // --- 角度锁定与底盘控制优化 ---
    // private Follower follower; // <-- 4. 移除 Follower 成员变量
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;

    // 角度控制状态
    private boolean isAngleLockActive = false; // 是否开启锁定模式
    private double lockTargetHeading = 0;      // 目标角度 (弧度)

    // 调优后的转向参数
    private static final double TURN_P = 1; // 一个更平滑的P值，防止抖动
    private static final double TURN_DEADZONE_RAD = Math.toRadians(0.3); // 1度以内不纠偏，省电关键
    private static final double JOYSTICK_DEADZONE = 0.05; // 摇杆死区

    @Override
    public void runOpMode() {
        initializeHardware();

        // 开启批量读取模式以优化性能
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* <-- 5. 移除所有 Follower 初始化代码
        // 初始化 PedroPathing Follower (用于获取坐标)
        follower = ConstantsFar.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive(false);
        */

        waitForStart();
        runtime.reset();
        shooterPidTimer.reset();

        while (opModeIsActive()) {
            /* <-- 6. 移除 follower.update() 调用，这是性能提升的关键
            // 1. 必须调用 update 来刷新里程计
            follower.update();
            */

            // 2. 处理按键逻辑 (LB重置 / RB锁定)
            handleAngleButtons();

            // 3. 底盘控制 (包含严格的逻辑隔离)
            handleManualControlWithLock();

            // 4. 处理核心系统（射击 + 进弹）
            handleShooterSystem();

            // 5. 更新遥测数据
            updateTelemetry();
        }
    }

    /**
     * 新增辅助函数：直接从IMU获取机器人当前的航向角，并转换为弧度。
     * 这是本次修改的核心，用轻量级操作替代重量级的里程计。
     * @return 当前航向角，单位为弧度 (-PI 到 PI)。
     */
    private double getHeadingInRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private void handleAngleButtons() {
        // [Left Bumper] 重置所有航向 (现在只重置 IMU)
        if (gamepad1.left_bumper && !previousLeftBumper) {
            imu.resetYaw();
            // follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0)); // <-- 7. 移除对 Follower 的操作
            // 重置时必须关闭锁定，防止机器突然旋转
            isAngleLockActive = false;
        }
        previousLeftBumper = gamepad1.left_bumper;

        // [Right Bumper] 切换锁定模式 (Toggle)
        if (gamepad1.right_bumper && !previousRightBumper) {
            isAngleLockActive = !isAngleLockActive; // 切换开关

            if (isAngleLockActive) {
                // 如果开启，设置目标为 -114 度 (转换为弧度)
                lockTargetHeading = Math.toRadians(-114);
            }
        }
        previousRightBumper = gamepad1.right_bumper;
    }

    private void handleManualControlWithLock() {
        // 1. 获取摇杆输入
        double y = -gamepad1.left_stick_y; // 前后
        double x = gamepad1.left_stick_x;  // 左右
        double yaw = gamepad1.right_stick_x; // 手动旋转

        // 2. 应用死区
        if (Math.abs(y) < JOYSTICK_DEADZONE) y = 0;
        if (Math.abs(x) < JOYSTICK_DEADZONE) x = 0;
        if (Math.abs(yaw) < JOYSTICK_DEADZONE) yaw = 0;

        // 3. 计算最终旋转力度 (Turn Power)
        double rotationPower;

        // --- 核心隔离逻辑 ---
        // 情况 A: 驾驶员正在手动旋转 (手动优先)
        if (yaw != 0) {
            rotationPower = yaw;
        }
        // 情况 B: 驾驶员没动摇杆，且锁定模式已开启 (PID 介入)
        else if (isAngleLockActive) {
            // double currentHeading = follower.getPose().getHeading(); // <-- 8. 旧代码
            double currentHeading = getHeadingInRadians(); // <-- 9. 新代码：使用轻量级的IMU
            double error = getAngleDifference(lockTargetHeading, currentHeading);

            // 优化：如果误差在死区内，则不进行调整，彻底停止电机抖动
            if (Math.abs(error) < TURN_DEADZONE_RAD) {
                rotationPower = 0;
            } else {
                // P 控制器计算转向力 (注意：可能需要根据你的电机/底盘方向调整正负号)
                rotationPower = Range.clip(-error * TURN_P, -0.8, 0.8);
            }
        }
        // 情况 C: 没动摇杆，且没开锁定 (完全静止)
        else {
            rotationPower = 0; // 强制为0，杜绝任何PID干扰
        }

        // 4. 场地方向转换 (Field Centric)
        // double robotHeading = follower.getPose().getHeading(); // <-- 10. 旧代码
        double robotHeading = getHeadingInRadians(); // <-- 11. 新代码：使用轻量级的IMU
        double rotX = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
        double rotY = x * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);

        // 5. 麦轮运动学解算与归一化
        // 这种归一化方法可以在保持移动方向的同时，确保总功率不超过1.0，从而获得最大推力
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationPower), 1.0);
        double frontLeftPower = (rotY + rotX + rotationPower) / denominator;
        double backLeftPower = (rotY - rotX + rotationPower) / denominator;
        double frontRightPower = (rotY - rotX - rotationPower) / denominator;
        double backRightPower = (rotY + rotX - rotationPower) / denominator;

        // 6. 设置电机功率
        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    /**
     * 计算两个角度之间的最小差值 (-PI 到 PI)，确保走最短路径
     */
    private double getAngleDifference(double target, double current) {
        double difference = target - current;
        while (difference < -Math.PI) difference += 2 * Math.PI;
        while (difference > Math.PI) difference -= 2 * Math.PI;
        return difference;
    }

    private void initializeHardware() {
        // 驱动电机
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBehindDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBehindDrive");

        // 功能电机
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "MOZART");

        // 双发射电机
        shooterMotor = hardwareMap.get(DcMotorEx.class, "SH");
        shooterHelper = hardwareMap.get(DcMotorEx.class, "HS");

        imu = hardwareMap.get(IMU.class, "imu");
        holdServo = hardwareMap.get(CRServo.class, "Hold");
        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");

        // 传感器
        jujuSensor = hardwareMap.get(DigitalChannel.class, "juju");
        jujuSensor.setMode(DigitalChannel.Mode.INPUT);
        bigjuju = hardwareMap.get(DigitalChannel.class, "bigjuju");
        bigjuju.setMode(DigitalChannel.Mode.INPUT);

        // 电机方向
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        blender.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterHelper.setDirection(DcMotor.Direction.FORWARD);

        // 电机零功率行为
        DcMotor.ZeroPowerBehavior brakeBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        leftFrontDrive.setZeroPowerBehavior(brakeBehavior);
        leftBackDrive.setZeroPowerBehavior(brakeBehavior);
        rightFrontDrive.setZeroPowerBehavior(brakeBehavior);
        rightBackDrive.setZeroPowerBehavior(brakeBehavior);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterHelper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 电机运行模式
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterHelper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 舵机初始位置
        servoLP.setPosition(0.0);
        servoRP.setPosition(0.925);

        // IMU 初始化
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.addData("Status", "Initialized: Final Optimized Version");
        telemetry.update();
    }

    private void handleShooterSystem() {
        updateShooterSpeed();

        // 按键状态切换
        if (gamepad1.x) {
            isManualStopped = true; state = false; intakeMode = false;
            stopFunctionalMotors();
            TARGET_RPM = 2680; ErrorRange = 2680;
            servoLP.setPosition(0.56); servoRP.setPosition(0.365);
        }
        if (gamepad1.a) {
            intakeMode = true; state = false; isManualStopped = false;
            TARGET_RPM = 1500;
        }
        if (gamepad1.b) {
            intakeMode = false; state = false;
            intake.setPower(1); holdServo.setPower(-1); blender.setPower(1);
        }
        boolean currentYState = gamepad1.y;
        if (currentYState && !lastYState) {
            state = true; intakeMode = false; isManualStopped = false;
        }
        lastYState = currentYState;

        handleDpadControls();

        // 模式执行
        if (intakeMode) {
            intake.setPower(1);
            holdServo.setPower(1);
            boolean isObjectDetected = jujuSensor.getState() || bigjuju.getState();
            blender.setPower(isObjectDetected ? 0 : 0.5);
        } else if (state && !isManualStopped) {
            handleShooterTriggerLogic();
        }
    }

    private void updateShooterSpeed() {
        if (TARGET_RPM <= 0) {
            shooterMotor.setPower(0); shooterHelper.setPower(0);
            shooterLastError = 0; return;
        }
        double targetVelTPS = (TARGET_RPM * MOTOR_TICK_COUNT) / 60.0;
        double currentVelTPS = shooterMotor.getVelocity();
        double errorRPM = targetVelTPS - currentVelTPS;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt == 0) dt = 1e-9;
        double derivative = (errorRPM - shooterLastError) / dt;
        shooterLastError = errorRPM;
        double feedForward = SHOOTER_F * targetVelTPS;
        double pidPower = (SHOOTER_P * errorRPM) + (SHOOTER_D * derivative);
        double finalPower = Range.clip(feedForward + pidPower, -1.0, 1.0);
        shooterMotor.setPower(finalPower);
        shooterHelper.setPower(finalPower);
    }

    private void handleShooterTriggerLogic() {
        double currentRPM = (shooterMotor.getVelocity() / MOTOR_TICK_COUNT) * 60;
        boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;
        boolean isHighSpeedGear = TARGET_RPM > 3300;

        if (!isHighSpeedGear || velocityCheck) {
            intake.setPower(1); holdServo.setPower(1);
            if (Math.abs(TARGET_RPM - 3400) < 50) blender.setPower(0.7);
            else if (Math.abs(TARGET_RPM - 3050) < 50) blender.setPower(0.85);
            else blender.setPower(1);
        } else {
            intake.setPower(0); blender.setPower(0); holdServo.setPower(0);
        }
    }

    private void handleDpadControls() {
        if (gamepad1.dpad_down) { TARGET_RPM = 2100; ErrorRange = 2100; servoLP.setPosition(0); servoRP.setPosition(0.925); }
        if (gamepad1.dpad_left) { TARGET_RPM = 2720; ErrorRange = 2720; servoLP.setPosition(0.56); servoRP.setPosition(0.365); }
        if (gamepad1.dpad_up) { TARGET_RPM = 3050; ErrorRange = 60; servoLP.setPosition(0.76); servoRP.setPosition(0.165); }
        if (gamepad1.dpad_right) { TARGET_RPM = 3400; ErrorRange = 470; servoLP.setPosition(0.925); servoRP.setPosition(0.03); }
    }

    private void stopFunctionalMotors() {
        intake.setPower(0); blender.setPower(0); holdServo.setPower(0);
        shooterMotor.setPower(0); shooterHelper.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("--- Chassis Control ---", "");
        telemetry.addData("Angle Lock (RB)", isAngleLockActive ? "LOCKED [-114]" : "FREE");
        // telemetry.addData("Heading (Deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading())); // <-- 12. 旧代码
        telemetry.addData("Heading (Deg)", "%.1f", Math.toDegrees(getHeadingInRadians())); // <-- 13. 新代码：使用轻量级的IMU

        telemetry.addData("--- Robot Mode ---", "");
        if (intakeMode) telemetry.addData("Mode", "INTAKE");
        else if (state) telemetry.addData("Mode", "SHOOTING");
        else telemetry.addData("Mode", "STANDBY");

        telemetry.addData("--- Shooter System ---", "");
        telemetry.addData("Shooter Target", "%.0f RPM", TARGET_RPM);
        telemetry.addData("Shooter Actual", "%.0f RPM", (shooterMotor.getVelocity() / MOTOR_TICK_COUNT) * 60);

        telemetry.update();
    }
}