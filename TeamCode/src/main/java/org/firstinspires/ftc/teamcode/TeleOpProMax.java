package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@TeleOp(name="TeleOpProMax", group="Linear Opmode")
public class TeleOpProMax extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;

    // 传感器
    private DigitalChannel jujuSensor = null;
    private DigitalChannel bigjuju = null; // [新增] 第二个传感器

    // 电机与舵机
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor intake = null;
    private DcMotor blender = null;

    // 双发射电机
    private DcMotorEx shooterMotor = null;  // 主电机 "SH"
    private DcMotorEx shooterHelper = null; // 副电机 "HS"

    private Servo servoRP = null;
    private Servo servoLP = null;
    private CRServo holdServo = null;

    // 状态变量
    private boolean lastYState = false;
    private boolean state = false; // true = 射击模式
    private boolean intakeMode = false; // true = 进弹模式
    private boolean lastLeftBumperState = false;
    private boolean isManualStopped = false;

    private ElapsedTime blenderTimer = new ElapsedTime();
    private boolean blenderRotating = false;
    private boolean lastRightBumperState = false;

    // PID与常量
    public static final double MOTOR_TICK_COUNT = 28;
    public static double TARGET_RPM = 0;
    public static boolean normalState = true;
    public static int ErrorRange = 50;

    // 自定义PID参数
    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.006;
    private static final double SHOOTER_F = 0.0004;
    private static final double SHOOTER_D = 0.00001;

    // 底盘PID参数
    private final double TURN_POWER = 1;
    private final double HEADING_THRESHOLD = 2.0;
    private final double P_TURN_GAIN = 0.035;
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private final double I_GAIN = 0.000;
    private final double D_GAIN = 0.0015;

    private final double TARGET_ANGLE_LEFT_TRIGGER = -127.0;
    private final double TARGET_ANGLE_RIGHT_TRIGGER = 127.0;
    private static final double JOYSTICK_DEADZONE = 0.05;
    private static final double TRIGGER_THRESHOLD = 0.3;

    @Override
    public void runOpMode() {
        initializeHardware();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        runtime.reset();
        previousTime = System.nanoTime();
        shooterPidTimer.reset();

        while (opModeIsActive()) {
            double heading = getHeading();
            handleHeadingReset();

            // 底盘控制逻辑
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;

            if (leftTrigger > TRIGGER_THRESHOLD) {
                performAutoTurn(TARGET_ANGLE_LEFT_TRIGGER);
            } else if (rightTrigger > TRIGGER_THRESHOLD) {
                performAutoTurn(TARGET_ANGLE_RIGHT_TRIGGER);
            } else {
                resetPID();
                handleManualControl(heading);
            }

            // 处理核心系统（射击 + 进弹 + 双传感器逻辑）
            handleShooterSystem();
            updateTelemetry();
        }
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

        // [修改] 两个传感器初始化
        jujuSensor = hardwareMap.get(DigitalChannel.class, "juju");
        jujuSensor.setMode(DigitalChannel.Mode.INPUT);
        bigjuju = hardwareMap.get(DigitalChannel.class, "bigjuju");
        bigjuju.setMode(DigitalChannel.Mode.INPUT);

        // 舵机初始位置
        servoLP.setPosition(0.0);
        servoRP.setPosition(0.925);

        configureDriveMotors();
        configureFunctionalMotors();
        initializeIMU();

        telemetry.addData("Status", "Initialized Dual Shooter & Dual Sensor");
        telemetry.update();
    }

    private void configureDriveMotors() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        DcMotor.ZeroPowerBehavior brakeBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        leftFrontDrive.setZeroPowerBehavior(brakeBehavior);
        leftBackDrive.setZeroPowerBehavior(brakeBehavior);
        rightFrontDrive.setZeroPowerBehavior(brakeBehavior);
        rightBackDrive.setZeroPowerBehavior(brakeBehavior);
    }

    private void configureFunctionalMotors() {
        intake.setDirection(DcMotor.Direction.REVERSE);
        blender.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterHelper.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterHelper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterHelper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(500);
    }

    private void handleShooterSystem() {
        // 1. 更新双电机速度控制
        updateShooterSpeed();
        handleBlenderTimer();

        // 2. 按键状态切换
        if (gamepad1.x) {
            isManualStopped = true;
            state = false;
            intakeMode = false;
            stopFunctionalMotors();
            normalState = false;
            TARGET_RPM = 2680;
            ErrorRange = 2680;
            servoLP.setPosition(0.56);
            servoRP.setPosition(0.365);
        }

        if (gamepad1.a) {
            intakeMode = true;
            state = false;
            isManualStopped = false;
            TARGET_RPM = 1500;
        }

        if (gamepad1.b) {
            intakeMode = false;
            state = false;
            intake.setPower(1);
            holdServo.setPower(-1);
            blender.setPower(1);
        }

        boolean currentYState = gamepad1.y;
        if (currentYState && !lastYState) {
            state = true;
            intakeMode = false;
            isManualStopped = false;
        }
        lastYState = currentYState;

        handleDpadControls();

        // =========================================================
        // 模式执行部分
        // =========================================================

        // A. 执行进弹模式 (Intake Mode) - 这里包含双传感器逻辑
        if (intakeMode) {
            intake.setPower(1);
            holdServo.setPower(1);

            // [修改] 任意一个检测到球，结果就是 True
            boolean isObjectDetected = jujuSensor.getState() || bigjuju.getState();

            if (isObjectDetected) {
                // 检测到物体 -> 停止 Blender
                blender.setPower(0);
            } else {
                // 未检测到 -> Blender 转动
                blender.setPower(0.5);
            }
        }
        // B. 执行射击模式
        else if (state && !isManualStopped) {
            handleShooterTriggerLogic();
        }
    }

    private void updateShooterSpeed() {
        if (TARGET_RPM <= 0) {
            shooterMotor.setPower(0);
            shooterHelper.setPower(0);
            shooterLastError = 0;
            return;
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
        double currentVelocityTicks = shooterMotor.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;

        boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;
        boolean isHighSpeedGear = TARGET_RPM > 3300;

        if (!isHighSpeedGear || velocityCheck) {
            intake.setPower(1);
            // 根据不同的RPM设置不同的blender功率
            if (Math.abs(TARGET_RPM - 3400) < 50) {  // dpad_right (3400 RPM)
                blender.setPower(0.7);
            } else if (Math.abs(TARGET_RPM - 3050) < 50) {  // dpad_up (3050 RPM)
                blender.setPower(0.85);
            } else {  // dpad_down (2100 RPM) 和 dpad_left (2720 RPM)
                blender.setPower(1);
            }

            holdServo.setPower(1);
        } else {
            intake.setPower(0);
            blender.setPower(0);
            holdServo.setPower(0);
        }
    }

    private void handleBlenderTimer() {
        boolean currentRightBumper = gamepad1.right_bumper;
        if (currentRightBumper && !lastRightBumperState && !isManualStopped) {
            blenderRotating = true;
            blenderTimer.reset();
        }
        lastRightBumperState = currentRightBumper;
        if (blenderRotating) {
            if (blenderTimer.seconds() < 0.2) {
                blender.setPower(0.8);
            } else {
                blender.setPower(0);
                blenderRotating = false;
            }
        }
    }

    private void handleDpadControls() {
        if (gamepad1.dpad_down) {
            normalState = false;
            TARGET_RPM = 2100;
            ErrorRange = 2100;
            servoLP.setPosition(0);
            servoRP.setPosition(0.925);
        }
        if (gamepad1.dpad_left) {
            normalState = false;
            TARGET_RPM = 2720;
            ErrorRange = 2720;
            servoLP.setPosition(0.56);
            servoRP.setPosition(0.365);
        }
        if (gamepad1.dpad_up) {
            normalState = false;
            TARGET_RPM = 3050;
            ErrorRange = 60;
            servoLP.setPosition(0.76);
            servoRP.setPosition(0.165);
        }
        if (gamepad1.dpad_right) {
            normalState = false;
            TARGET_RPM = 3400;
            ErrorRange = 470;
            servoLP.setPosition(0.925);
            servoRP.setPosition(0.03);
        }
    }

    private void updateTelemetry() {
        double currentVelocityTicks = shooterMotor.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        if (intakeMode) telemetry.addData("Mode", "INTAKE (Sensors Active)");
        else if (state) telemetry.addData("Mode", "SHOOTING");
        else telemetry.addData("Mode", "STANDBY / MANUAL");

        if (jujuSensor != null && bigjuju != null) {
            boolean s1 = jujuSensor.getState();
            boolean s2 = bigjuju.getState();
            // 显示两个传感器的实时数值
            telemetry.addData("Sensors", "Juju:%b | BigJuju:%b", s1, s2);
            telemetry.addData("Ball Status", (s1 || s2) ? "DETECTED (Stop)" : "CLEAR (Run)");
        }

        telemetry.addData("Shooter", "T:%.0f | C:%.2f", TARGET_RPM, currentRPM);
        telemetry.update();
    }

    // -----------------------------------------------------------------------------------------
    // 底盘移动相关函数
    // -----------------------------------------------------------------------------------------

    private void handleHeadingReset() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumperState) {
            imu.resetYaw();
            sleep(100);
        }
        lastLeftBumperState = currentLeftBumper;
    }

    private void performAutoTurn(double targetAngle) {
        double currentHeading = getHeading();
        double headingError = shortestPathAngleError(currentHeading, targetAngle);

        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            stopDriveMotors();
            return;
        }

        double turnPower = calculatePIDOutput(headingError);
        setMecanumPower(0, 0, turnPower);
    }

    private double shortestPathAngleError(double currentAngle, double targetAngle) {
        currentAngle = normalizeAngle(currentAngle);
        targetAngle = normalizeAngle(targetAngle);
        double rawError = targetAngle - currentAngle;
        return normalizeAngle(rawError);
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    private void handleManualControl(double heading) {
        double headingRad = Math.toRadians(heading);
        double rawAxial = -gamepad1.left_stick_y;
        double rawLateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        rawAxial = applyDeadzone(rawAxial, JOYSTICK_DEADZONE);
        rawLateral = applyDeadzone(rawLateral, JOYSTICK_DEADZONE);
        yaw = applyDeadzone(yaw, JOYSTICK_DEADZONE);

        double fieldAxial = rawAxial * Math.cos(headingRad) - rawLateral * Math.sin(headingRad);
        double fieldLateral = rawAxial * Math.sin(headingRad) + rawLateral * Math.cos(headingRad);

        double leftFrontPower = fieldAxial + fieldLateral + yaw;
        double rightFrontPower = fieldAxial - fieldLateral - yaw;
        double leftBackPower = fieldAxial - fieldLateral + yaw;
        double rightBackPower = fieldAxial + fieldLateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9;
        if (deltaTime == 0) deltaTime = 0.01;

        double proportional = P_TURN_GAIN * error;
        integralSum += error * deltaTime;
        double maxIntegral = 0.5 / (I_GAIN + 1e-9);
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        double integral = I_GAIN * integralSum;
        double derivative = D_GAIN * (error - previousError) / deltaTime;

        double output = Range.clip(proportional + integral + derivative, -TURN_POWER, TURN_POWER);
        previousError = error;
        previousTime = currentTime;
        return output;
    }

    private void resetPID() {
        previousError = 0;
        integralSum = 0;
        previousTime = System.nanoTime();
    }

    private void setMecanumPower(double drive, double strafe, double turn) {
        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

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

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void stopFunctionalMotors() {
        intake.setPower(0);
        blender.setPower(0);
        holdServo.setPower(0);
        shooterMotor.setPower(0);
        shooterHelper.setPower(0);
    }

    private double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) return 0;
        return (value - Math.copySign(deadzone, value)) / (1 - deadzone);
    }
}