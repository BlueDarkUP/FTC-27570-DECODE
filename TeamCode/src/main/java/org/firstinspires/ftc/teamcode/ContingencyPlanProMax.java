package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

@TeleOp
public class ContingencyPlanProMax extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private IMU imu;
    private RevColorSensorV3 ballSensor = null;
    private DistanceSensor distanceSensor = null;
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

    private boolean lastYState = false;
    private boolean state = false;
    private boolean isMozartBraked = false;
    private boolean lastLeftBumperState = false;
    private boolean isManualStopped = false;

    public static final double MOTOR_TICK_COUNT = 28;
    public static double P = 250, I = 0.1, D = 30, F = 13;
    public static double TARGET_RPM = 0;
    public static int ErrorRange = 50;

    private double targetHeading = 0;
    private final double TURN_POWER = 1;
    private final double HEADING_THRESHOLD = 2.0;
    private final double P_TURN_GAIN = 0.025;
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime = 0;
    private final double I_GAIN = 0.000;
    private final double D_GAIN = 0.0;

    private final double TARGET_ANGLE_RIGHT = -135.0;
    private static final double MIN_SERVO_POS = 0.0;
    private static final double MAX_SERVO_POS = 1.0;
    private static final double MIN_ANGLE_DEG = 40.0;
    private static final double MAX_ANGLE_DEG = 67.44;

    private static final double JOYSTICK_DEADZONE = 0.05;

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

        while (opModeIsActive()) {
            double heading = getHeading();
            if(gamepad1.right_stick_button){
                AJI.setPosition(0.9);
                MIDE.setPosition(0);
            }

            handleHeadingReset();

            if (gamepad1.right_bumper) {
                performAutoTurn();
            } else {
                resetPID();
                handleManualControl(heading);
            }

            handleShooterSystem();

            updateTelemetry();
        }
    }

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

        distanceSensor = hardwareMap.get(DistanceSensor.class, "juju");

        servoRP.setPosition(0.91);
        servoLP.setPosition(0.078);
        AJI.setPosition(0);
        MIDE.setPosition(0.9);

        configureDriveMotors();
        configureFunctionalMotors();
        initializeIMU();
        configureShooterPID();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

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

    private void configureFunctionalMotors() {
        intake.setDirection(DcMotor.Direction.FORWARD);
        blender.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior brakeBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        intake.setZeroPowerBehavior(brakeBehavior);
        blender.setZeroPowerBehavior(brakeBehavior);
        shooter.setZeroPowerBehavior(brakeBehavior);
    }

    private void initializeIMU() {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(500);
    }

    private void configureShooterPID() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    private void handleHeadingReset() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumperState) {
            imu.resetYaw();
            sleep(100);
        }
        lastLeftBumperState = currentLeftBumper;
    }

    private void performAutoTurn() {
        double currentHeading = getHeading();
        double headingError = currentHeading - TARGET_ANGLE_RIGHT;

        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            stopDriveMotors();
            return;
        }

        double turnPower = calculatePIDOutput(headingError);
        setMecanumPower(0, 0, turnPower);
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
        if (max > 1.0 && max > 0) {
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

    private void handleShooterSystem() {
        double targetTicksPerSecond = TARGET_RPM * MOTOR_TICK_COUNT / 60;
        shooter.setVelocity(targetTicksPerSecond);

        // --- 1. 优先读取传感器数据 ---
        NormalizedRGBA colors = ballSensor.getNormalizedColors();
        boolean isColorDetected = (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1);
        double currentDistance = distanceSensor.getDistance(DistanceUnit.MM);
        boolean isDistanceDetected = (currentDistance < 50);

        boolean isObjectDetected = isColorDetected || isDistanceDetected;

        // --- 2. 传感器刹车逻辑 ---
        if (!state && !isManualStopped) {
            // 如果检测到物体，立即置为刹车状态
            if (isObjectDetected) {
                isMozartBraked = true;
            }
        }

        // --- 3. 处理按键 A (Cross) ---
        if (gamepad1.a) {
            state = false;
            isManualStopped = false;

            // 【关键修改】：只有当传感器没检测到物体时，才允许解除刹车
            // 这样如果你按住A，但传感器前面有球，MOZART 依然会保持刹车状态
            if (!isObjectDetected) {
                isMozartBraked = false;
            }

            intake.setPower(1);
            // blender.setPower(1); // 删除这行，统一在下面根据刹车状态控制
            washer.setPower(1);
            classifyServo.setPower(1);
            holdServo.setPower(1);
            TARGET_RPM = 1500;
        }

        // --- 4. 统一控制 blender (MOZART) 电机 ---
        if (!state && !isManualStopped) {
            // 根据刹车标志位控制电机
            blender.setPower(isMozartBraked ? 0.0 : 1.0);
        }

        // --- 5. 其他按键逻辑 ---
        if (gamepad1.b) {
            // B键通常作为反转或强制排异，这里保留强制开启
            intake.setPower(1);
            blender.setPower(1);
            washer.setPower(1);
            classifyServo.setPower(1);
            holdServo.setPower(-1);
        }

        if (gamepad1.x) {
            isManualStopped = true;
            stopFunctionalMotors();
            TARGET_RPM = 2400;
            ErrorRange = 150;
            setLauncherServos(0.3);
            isMozartBraked = false;
        }

        handleDpadControls();
        handleShooterTrigger();
    }

    private void handleDpadControls() {
        if (gamepad1.dpad_right) {
            TARGET_RPM = 3280;
            ErrorRange = 100;
            setLauncherServos(0);
        }
        if (gamepad1.dpad_left) {
            TARGET_RPM = 2400;
            ErrorRange = 200;
            setLauncherServos(0.3);
        }
        if (gamepad1.dpad_down) {
            TARGET_RPM = 2000;
            ErrorRange = 200;
            setLauncherServos(1);
        }
        if (gamepad1.dpad_up) {
            TARGET_RPM = 2800;
            ErrorRange = 200;
            setLauncherServos(0.2);
        }
    }

    private void handleShooterTrigger() {
        double currentVelocityTicks = shooter.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;
        boolean isHighSpeedGear = TARGET_RPM > 3000;

        boolean currentYState = gamepad1.y;
        if (currentYState && !lastYState) {
            state = true;
            isManualStopped = false;
        }
        lastYState = currentYState;

        if (state && !isManualStopped) {
            if (!isHighSpeedGear || velocityCheck) {
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
    }

    private void updateTelemetry() {
        double currentVelocityTicks = shooter.getVelocity();
        double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
        boolean velocityCheck = Math.abs(currentRPM - TARGET_RPM) <= ErrorRange;
        double currentDistance = distanceSensor.getDistance(DistanceUnit.MM);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Heading", "%.1f", getHeading());
        telemetry.addData("Mode", gamepad1.right_bumper ? "Auto Turn" : "Manual");
        telemetry.addData("Shooter", "T:%.0f | C:%.2f", TARGET_RPM, currentRPM);
        telemetry.addData("Dist(mm)", "%.1f", currentDistance);
        telemetry.addData("Ready", velocityCheck ? "Yes" : "No");
        telemetry.addData("Stop", isManualStopped ? "Yes" : "No");
        telemetry.addData("Blender", blender.getPower() == 0 ? "Stop" : "Run");
        telemetry.update();
    }

    private double calculatePIDOutput(double error) {
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - previousTime) / 1e9;
        if (deltaTime == 0) deltaTime = 0.01;

        double proportional = P_TURN_GAIN * error;

        integralSum += error * deltaTime;
        double maxIntegral = 0.5 / I_GAIN;
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
        if (maxPower > 1.0 && maxPower > 0) {
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
        washer.setPower(0);
        classifyServo.setPower(0);
        holdServo.setPower(0);
    }

    private double calculateHeadingError(double currentHeading, double targetHeading) {
        double error = targetHeading - currentHeading;
        return error;
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void setLauncherServos(double servoPos) {
        double clampedPos = Range.clip(servoPos, MIN_SERVO_POS, MAX_SERVO_POS);
        servoRP.setPosition(clampedPos);
        servoLP.setPosition(1 - clampedPos);
    }

    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return (value - Math.copySign(deadzone, value)) / (1 - deadzone);
    }
}