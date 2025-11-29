package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class ContingencyPlan extends LinearOpMode {
    // 硬件声明
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private RevColorSensorV3 ballSensor = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intake = null;
    private DcMotor blender = null;
    private DcMotorEx shooter = null;
    private Servo servoRP = null;
    private Servo servoLP = null;
    private CRServo holdServo = null;
    private CRServo classifyServo = null;
    private CRServo washer = null;

    // 状态变量
    private boolean lastYState = false;
    private boolean state = false;
    private boolean isMozartBraked = false; // 修复作用域问题
    private boolean lastLeftBumperState = false; // 左肩键上一帧状态（用于上升沿检测）
    private boolean isManualStopped = false; // 新增：手动停止标志

    // 电机参数
    public static final double MOTOR_TICK_COUNT = 28;
    public static double P = 135, I = 0, D = 80, F = 14;
    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;

    // 转向控制参数
    private double targetHeading = 0;
    private boolean isTurningToTarget = false;
    private final double TURN_POWER = 0.5;
    private final double HEADING_THRESHOLD = 2.0;
    private final double P_TURN_GAIN = 0.07;
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private final double I_GAIN = 0.000;
    private final double D_GAIN = 0.005;

    // 角度常量（舵机角度范围：0-1对应物理角度40-67.44度）
    private final double TARGET_ANGLE_LEFT = -45.0;
    private final double TARGET_ANGLE_RIGHT = 45.0;
    private static final double MIN_SERVO_POS = 0.0;    // 舵机最小位置
    private static final double MAX_SERVO_POS = 1.0;    // 舵机最大位置
    private static final double MIN_ANGLE_DEG = 40.0;   // 对应物理最小角度
    private static final double MAX_ANGLE_DEG = 67.44;  // 对应物理最大角度

    // 摇杆死区（防止误触）
    private static final double JOYSTICK_DEADZONE = 0.05;

    @Override
    public void runOpMode() {
        // 硬件初始化
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBehindDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBehindDrive");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blender = hardwareMap.get(DcMotor.class, "MOZART");
        shooter = hardwareMap.get(DcMotorEx.class, "SH");
        imu = hardwareMap.get(IMU.class, "imu");
        holdServo = hardwareMap.get(CRServo.class, "Hold");
        classifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");

        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");
        ballSensor = hardwareMap.get(RevColorSensorV3.class, "color"); // 修复重复初始化

        // 舵机初始位置
        servoRP.setPosition(0.91);
        servoLP.setPosition(0.078);

        // 驱动电机配置
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 功能电机配置
        intake.setDirection(DcMotor.Direction.FORWARD);
        blender.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU初始化
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(500);

        // 射手电机PID配置
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // PID时间戳初始化
        previousTime = System.nanoTime();

        // 初始化提示
        telemetry.addData("Status", "Initialized");
        telemetry.addData("无头模式", "已启用 | 左肩键重置方向基准");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            washer = hardwareMap.get(CRServo.class, "washer");
            // -------------------------- 关键修正：提前定义heading变量，确保全局可见 --------------------------
            double heading = getHeading(); // 移到循环开头，所有代码块都能访问

            // -------------------------- 1. 左肩键重置无头模式基准 --------------------------
            boolean currentLeftBumper = gamepad1.left_bumper;
            if (currentLeftBumper && !lastLeftBumperState) {
                imu.resetYaw();
                heading = 0; // 重置后立即更新heading值
                telemetry.addData("无头模式", "基准方向已重置");
                sleep(100); // 防抖动
            }
            lastLeftBumperState = currentLeftBumper;

            // -------------------------- 2. 自动转向逻辑（右肩键触发） --------------------------
            if (gamepad1.right_bumper && !isTurningToTarget) {
                targetHeading = TARGET_ANGLE_RIGHT;
                isTurningToTarget = true;
                resetPID();
                telemetry.addData("自动转向", "转向 45 度");
            }

            // -------------------------- 3. 无头模式移动控制 --------------------------
            if (isTurningToTarget) {
                // 自动转向时执行转向控制
                performAutoTurn();
            } else {
                // 无头模式（场地中心）移动计算
                // 航向角转弧度（用于坐标旋转）
                double headingRad = Math.toRadians(heading);

                // 读取摇杆输入并处理死区
                double rawAxial = -gamepad1.left_stick_y; // 前/后
                double rawLateral = gamepad1.left_stick_x; // 左/右
                double yaw = gamepad1.right_stick_x;      // 旋转

                // 死区处理
                rawAxial = applyDeadzone(rawAxial, JOYSTICK_DEADZONE);
                rawLateral = applyDeadzone(rawLateral, JOYSTICK_DEADZONE);
                yaw = applyDeadzone(yaw, JOYSTICK_DEADZONE);

                // 无头模式核心：旋转摇杆输入向量（相对于场地固定）
                double fieldAxial = rawAxial * Math.cos(headingRad) - rawLateral * Math.sin(headingRad);
                double fieldLateral = rawAxial * Math.sin(headingRad) + rawLateral * Math.cos(headingRad);

                // 麦克纳姆轮功率计算
                double leftFrontPower = fieldAxial + fieldLateral + yaw;
                double rightFrontPower = fieldAxial - fieldLateral - yaw;
                double leftBackPower = fieldAxial - fieldLateral + yaw;
                double rightBackPower = fieldAxial + fieldLateral - yaw;

                // 功率归一化（防止超过1）
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));
                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // 设置电机功率
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            // -------------------------- 4. 射手/进料系统控制 --------------------------
            // 目标RPM转编码器速度
            double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
            shooter.setVelocity(targetTicksPerSecond);

            // A键：启动进料/射击（重置手动停止标志）
            if (gamepad1.a) {
                state = false;
                isManualStopped = false; // 重置手动停止标志
                intake.setPower(1);
                blender.setPower(1);
                washer.setPower(1);
                classifyServo.setPower(1);
                holdServo.setPower(1);
                TARGET_RPM = 1500;
            }

            // 颜色传感器检测（停止搅拌机）- 仅在未手动停止时有效
            if (!state && !isManualStopped) {
                NormalizedRGBA colors = ballSensor.getNormalizedColors();
                if (!isMozartBraked) {
                    if (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1) {
                        isMozartBraked = true;
                    }
                }
                if (isMozartBraked) {
                    blender.setPower(0.0);
                } else {
                    blender.setPower(1.0);
                }
            }

            // B键：反向进料
            if (gamepad1.b) {
                intake.setPower(-0.8);
                blender.setPower(-0.55);
                shooter.setPower(0);
            }

            // X键：停止所有功能电机（关键修改）
            if (gamepad1.x) {
                isManualStopped = true; // 设置手动停止标志
                intake.setPower(0);
                blender.setPower(0); // 确保搅拌机停止
                washer.setPower(0);
                classifyServo.setPower(0);
                holdServo.setPower(0);
                TARGET_RPM = 1500;
                isMozartBraked = false; // 重置搅拌机状态
                telemetry.addData("系统状态", "所有功能已停止");
            }

            // 方向键：设置射击参数（不同距离）
            if (gamepad1.dpad_right) {
                TARGET_RPM = 3280; // 超远
                ErrorRange = 100;
                setLauncherServos(0); // 舵机位置（0-1）
            }
            if (gamepad1.dpad_left) {
                TARGET_RPM = 2400; // 三角腰
                ErrorRange = 150;
                setLauncherServos(0.7);
            }
            if (gamepad1.dpad_down) {
                TARGET_RPM = 2000; // 三角底部
                ErrorRange = 150;
                setLauncherServos(1);
            }
            if (gamepad1.dpad_up) {
                TARGET_RPM = 2800; // 三角顶点
                ErrorRange = 100;
                setLauncherServos(0.2);
            }

            // Y键：触发射击（速度达标后启动）
            double currentVelocityTicks = shooter.getVelocity();
            double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
            boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;

            boolean currentYState = gamepad1.y;
            if (currentYState && !lastYState) {
                state = true;
                isManualStopped = false; // 重置手动停止标志
            }
            lastYState = currentYState;

            // 速度达标则启动射击，否则停止
            if (state && !isManualStopped) {
                if (velocityCheck) {
                    intake.setPower(1);
                    blender.setPower(1);
                    washer.setPower(1);
                    classifyServo.setPower(1);
                    holdServo.setPower(1);
                } else {
                    intake.setPower(0);
                    blender.setPower(0);
                    washer.setPower(0);
                    classifyServo.setPower(0);
                    holdServo.setPower(0);
                }
            }

            // -------------------------- 5. 遥测数据更新 --------------------------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("无头模式-当前航向", "%.1f°", heading);
            telemetry.addData("目标航向", "%.1f°", targetHeading);
            telemetry.addData("自动转向状态", isTurningToTarget ? "进行中" : "关闭");
            telemetry.addData("射手RPM", "目标: %.0f | 当前: %.2f", TARGET_RPM, currentRPM);
            telemetry.addData("速度达标", velocityCheck ? "是" : "否");
            telemetry.addData("手动停止状态", isManualStopped ? "已停止" : "运行中");
            telemetry.addData("搅拌机状态", blender.getPower() == 0 ? "停止" : "运行");
            telemetry.update();
        }
    }

    /**
     * 执行自动转向到目标角度（PID控制）
     */
    private void performAutoTurn() {
        double currentHeading = getHeading();
        double headingError = calculateHeadingError(currentHeading, targetHeading);

        // 误差在阈值内则停止转向
        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            stopMotors();
            isTurningToTarget = false;
            telemetry.addData("自动转向", "完成 | 误差: %.1f°", headingError);
            return;
        }

        // 计算PID输出并执行转向
        double turnPower = calculatePIDOutput(headingError);
        setMecanumPower(0, 0, turnPower);
    }

    /**
     * PID控制输出计算
     */
    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9;
        if (deltaTime == 0) deltaTime = 0.01;

        // 比例项
        double proportional = P_TURN_GAIN * error;
        // 积分项（限幅防饱和）
        integralSum += error * deltaTime;
        double maxIntegral = 0.5 / I_GAIN;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;
        // 微分项
        double derivative = D_GAIN * (error - previousError) / deltaTime;

        // 总输出限幅
        double output = Range.clip(proportional + integral + derivative, -TURN_POWER, TURN_POWER);

        // 更新状态
        previousError = error;
        previousTime = currentTime;
        return output;
    }

    /**
     * 重置PID控制器状态
     */
    private void resetPID() {
        previousError = 0;
        integralSum = 0;
        previousTime = System.nanoTime();
    }

    /**
     * 设置麦克纳姆轮功率
     */
    private void setMecanumPower(double drive, double strafe, double turn) {
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        // 归一化
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * 停止所有驱动电机
     */
    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * 计算航向误差（标准化到-180~180°）
     */
    private double calculateHeadingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    /**
     * 获取当前航向（偏航角）
     */
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * 设置发射器舵机位置（修复原逻辑错误）
     * @param servoPos 舵机位置（0~1）
     */
    private void setLauncherServos(double servoPos) {
        // 限制舵机位置在0~1范围内
        double clampedPos = Range.clip(servoPos, MIN_SERVO_POS, MAX_SERVO_POS);

        // 根据舵机位置映射到物理角度（可选，仅用于调试）
        double physicalAngle = MIN_ANGLE_DEG + clampedPos * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);

        // 设置左右舵机位置（根据实际机械结构调整）
        servoRP.setPosition(clampedPos);
        servoLP.setPosition(1 - clampedPos); // 左舵机与右舵机反向

        telemetry.addData("发射器舵机", "位置: %.2f | 物理角度: %.1f°", clampedPos, physicalAngle);
    }

    /**
     * 摇杆死区处理
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        // 线性映射死区外的数值（保持0~1范围）
        return (value - Math.copySign(deadzone, value)) / (1 - deadzone);
    }
}