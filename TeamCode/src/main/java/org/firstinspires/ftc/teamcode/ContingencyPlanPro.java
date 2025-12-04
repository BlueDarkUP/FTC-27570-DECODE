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
public class ContingencyPlanPro extends LinearOpMode {
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
    private Servo AJI = null;
    private Servo MIDE = null;

    // 状态变量
    private boolean lastYState = false;
    private boolean state = false;
    private boolean isMozartBraked = false;
    private boolean lastLeftBumperState = false;
    private boolean isManualStopped = false;

    // 电机参数
    public static final double MOTOR_TICK_COUNT = 28;
    public static double P = 135, I = 0, D = 80, F = 14;
    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;

    // 角度常量
    private static final double MIN_SERVO_POS = 0.0;
    private static final double MAX_SERVO_POS = 1.0;

    // 摇杆死区
    private static final double JOYSTICK_DEADZONE = 0.05;

    @Override
    public void runOpMode() {
        // 硬件初始化
        initializeHardware();

        waitForStart();
        runtime.reset();

        // 主循环
        while (opModeIsActive()) {
            // 获取当前航向角
            double heading = getHeading();

            if(gamepad1.right_stick_button){
                AJI.setPosition(0.9);
                MIDE.setPosition(0);
            }

            // 1. 左肩键重置无头模式基准
            handleHeadingReset();

            // 2. 移动控制（始终为手动无头模式）
            handleManualControl(heading);

            // 3. 射手/进料系统控制
            handleShooterSystem();

            // 4. 遥测数据更新
            updateTelemetry();
        }
    }

    /**
     * 初始化所有硬件
     */
    private void initializeHardware() {
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
        washer = hardwareMap.get(CRServo.class, "washer");

        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");
        AJI = hardwareMap.get(Servo.class, "AJI");
        MIDE = hardwareMap.get(Servo.class, "MIDE");
        ballSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        // 舵机初始位置
        servoRP.setPosition(0.91);
        servoLP.setPosition(0.078);
        AJI.setPosition(0);
        MIDE.setPosition(0.9);

        // 驱动电机配置
        configureDriveMotors();

        // 功能电机配置
        configureFunctionalMotors();

        // IMU初始化
        initializeIMU();

        // 射手电机PID配置
        configureShooterPID();

        // 初始化提示
        telemetry.addData("Status", "Initialized");
        telemetry.addData("模式", "手动无头模式 | 左肩键重置方向");
        telemetry.update();
    }

    /**
     * 配置驱动电机
     */
    private void configureDriveMotors() {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior brakeBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        leftFrontDrive.setZeroPowerBehavior(brakeBehavior);
        leftBackDrive.setZeroPowerBehavior(brakeBehavior);
        rightFrontDrive.setZeroPowerBehavior(brakeBehavior);
        rightBackDrive.setZeroPowerBehavior(brakeBehavior);
    }

    /**
     * 配置功能电机
     */
    private void configureFunctionalMotors() {
        intake.setDirection(DcMotor.Direction.FORWARD);
        blender.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior brakeBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        intake.setZeroPowerBehavior(brakeBehavior);
        blender.setZeroPowerBehavior(brakeBehavior);
        shooter.setZeroPowerBehavior(brakeBehavior);
    }

    /**
     * 初始化IMU
     */
    private void initializeIMU() {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(500);
    }

    /**
     * 配置射手PID
     */
    private void configureShooterPID() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /**
     * 处理航向重置
     */
    private void handleHeadingReset() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumperState) {
            imu.resetYaw();
            telemetry.addData("无头模式", "基准方向已重置");
            sleep(100); // 防抖动
        }
        lastLeftBumperState = currentLeftBumper;
    }

    /**
     * 处理手动控制（无头模式）
     */
    private void handleManualControl(double heading) {
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
        if (max > 1.0 && max > 0) {
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

    /**
     * 处理射手系统
     */
    private void handleShooterSystem() {
        // 目标RPM转编码器速度
        double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
        shooter.setVelocity(targetTicksPerSecond);

        // A键：启动进料/射击
        if (gamepad1.a) {
            state = false;
            isManualStopped = false;
            isMozartBraked = false;
            intake.setPower(1);
            blender.setPower(1);
            washer.setPower(1);
            classifyServo.setPower(1);
            holdServo.setPower(1);
            TARGET_RPM = 1500;
            telemetry.addData("射手系统", "进料启动");
        }

        // 颜色传感器检测（停止搅拌机）- 仅在未手动停止时有效
        if (!state && !isManualStopped) {
            NormalizedRGBA colors = ballSensor.getNormalizedColors();
            if (!isMozartBraked) {
                // 建议：如果过于灵敏，可提高阈值，例如 > 0.02
                if (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1) {
                    isMozartBraked = true;
                    telemetry.addData("搅拌机", "检测到球体，停止搅拌");
                }
            }
            blender.setPower(isMozartBraked ? 0.0 : 1.0);
        }

        // B键：反向进料
        if (gamepad1.b) {
            intake.setPower(1);
            blender.setPower(1);
            washer.setPower(1);
            classifyServo.setPower(1);
            holdServo.setPower(-1);
            telemetry.addData("射手系统", "反向进料");
        }

        // X键：停止所有功能电机
        if (gamepad1.x) {
            isManualStopped = true;
            stopFunctionalMotors();
            TARGET_RPM = 2400;
            ErrorRange = 150;
            setLauncherServos(0.3);
            isMozartBraked = false;
            telemetry.addData("射手系统", "所有功能已停止");
        }

        // 方向键：设置射击参数
        handleDpadControls();

        // Y键：触发射击
        handleShooterTrigger();
    }

    /**
     * 处理方向键控制
     */
    private void handleDpadControls() {
        if (gamepad1.dpad_right) {
            TARGET_RPM = 3280;
            ErrorRange = 100;
            setLauncherServos(0);
            telemetry.addData("射手参数", "超远距离 | RPM: 3280");
        }
        if (gamepad1.dpad_left) {
            TARGET_RPM = 2400;
            ErrorRange = 200;
            setLauncherServos(0.3);
            telemetry.addData("射手参数", "三角腰部 | RPM: 2400");
        }
        if (gamepad1.dpad_down) {
            TARGET_RPM = 2000;
            ErrorRange = 200;
            setLauncherServos(1);
            telemetry.addData("射手参数", "三角底部 | RPM: 2000");
        }
        if (gamepad1.dpad_up) {
            TARGET_RPM = 2800;
            ErrorRange = 200;
            setLauncherServos(0.2);
            telemetry.addData("射手参数", "三角顶点 | RPM: 2800");
        }
    }

    /**
     * 处理射手触发
     */
    private void handleShooterTrigger() {
        double currentVelocityTicks = shooter.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;

        boolean currentYState = gamepad1.y;
        if (currentYState && !lastYState) {
            state = true;
            isManualStopped = false;
            telemetry.addData("射手触发", "准备射击");
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
                telemetry.addData("射手状态", "射击中 | RPM: %.1f", currentRPM);
            } else {
                intake.setPower(0);
                blender.setPower(0);
                washer.setPower(0);
                classifyServo.setPower(0);
                holdServo.setPower(0);
                telemetry.addData("射手状态", "等待速度达标 | 当前: %.1f | 目标: %.0f", currentRPM, TARGET_RPM);
            }
        }
    }

    /**
     * 更新遥测数据
     */
    private void updateTelemetry() {
        double currentVelocityTicks = shooter.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("当前航向", "%.1f°", getHeading());
        telemetry.addData("射手RPM", "目标: %.0f | 当前: %.2f", TARGET_RPM, currentRPM);
        telemetry.addData("速度达标", velocityCheck ? "是" : "否");
        telemetry.addData("手动停止", isManualStopped ? "已停止" : "运行中");
        telemetry.addData("搅拌机状态", blender.getPower() == 0 ? "停止" : "运行");
        telemetry.update();
    }

    /**
     * 停止功能电机
     */
    private void stopFunctionalMotors() {
        intake.setPower(0);
        blender.setPower(0);
        washer.setPower(0);
        classifyServo.setPower(0);
        holdServo.setPower(0);
    }

    /**
     * 获取当前航向（偏航角）
     */
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * 设置发射器舵机位置
     */
    private void setLauncherServos(double servoPos) {
        double clampedPos = Range.clip(servoPos, MIN_SERVO_POS, MAX_SERVO_POS);
        servoRP.setPosition(clampedPos);
        servoLP.setPosition(1 - clampedPos);
    }

    /**
     * 摇杆死区处理
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return (value - Math.copySign(deadzone, value)) / (1 - deadzone);
    }
}