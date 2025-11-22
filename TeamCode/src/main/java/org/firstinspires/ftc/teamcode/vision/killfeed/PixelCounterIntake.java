package org.firstinspires.ftc.teamcode.vision.killfeed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Queue;

/**
 * 这是一个封装了基于RPM的动态像素计数逻辑的模块。
 * 它可以被任何OpMode轻松调用，以实现进气和自动计数功能。
 */
public class PixelCounterIntake {

    // --- 硬件对象 ---
    private final DcMotorEx intakeMotor;
    private final DcMotor inhaleMotor;

    // --- 可配置常量 ---
    private static final double MOTOR_TICKS_PER_REV = 145.1;
    private static final double INTAKE_TARGET_RPM = 1150;
    private static final double P = 150;
    private static final double I = 10;
    private static final double D = 30;
    private static final double F = 25;
    private static final double INHALE_POWER = 0.55;

    // --- 计数逻辑常量 ---
    private static final int MOVING_AVERAGE_WINDOW_SIZE = 5;
    private static final double DROP_DETECTION_FACTOR = 0.95; // RPM下降到平均值的95%时触发
    private static final double RECOVERY_FACTOR = 0.80;       // RPM恢复到骤降前基线的80%时，认为恢复
    private static final double MINIMUM_RPM_FOR_DETECTION = 300; // RPM必须高于此值才开始检测
    private static final long COOLDOWN_DURATION_MS = 120;        // 冷却时间，防止重复计数

    // --- 内部状态变量 ---
    private int pixelCount = 0;
    private final Queue<Double> rpmHistory = new LinkedList<>();
    private double averageRpm = 0;
    private double baselineRpmBeforeDip = 0;
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private IntakeState currentState = IntakeState.IDLE;

    private enum IntakeState {
        IDLE,       // 空闲
        RUNNING,    // 正常运行
        DIP_DETECTED // 检测到骤降（冷却中）
    }

    /**
     * 构造函数
     * @param intake The main intake motor (with encoder).
     * @param inhale The secondary motor to pull pixels in.
     */
    public PixelCounterIntake(DcMotorEx intake, DcMotor inhale) {
        this.intakeMotor = intake;
        this.inhaleMotor = inhale;

        // 初始化硬件设置
        this.intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        this.inhaleMotor.setDirection(DcMotor.Direction.FORWARD);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.inhaleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 设置PIDF
        this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        this.intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    /**
     * 启动进气和计数系统。
     */
    public void start() {
        if (currentState == IntakeState.IDLE) {
            // 从空闲状态启动时，重置历史数据
            rpmHistory.clear();
            averageRpm = 0;
            currentState = IntakeState.RUNNING;
        }

        double targetVelocityTicksPerSecond = (INTAKE_TARGET_RPM / 60) * MOTOR_TICKS_PER_REV;
        intakeMotor.setVelocity(targetVelocityTicksPerSecond);
        inhaleMotor.setPower(INHALE_POWER);
    }

    /**
     * 停止进气和计数系统，并重置计数。
     */
    public void stop() {
        intakeMotor.setVelocity(0);
        inhaleMotor.setPower(0);

        if (currentState != IntakeState.IDLE) {
            // 切换到空闲状态时，重置像素计数
            pixelCount = 0;
            currentState = IntakeState.IDLE;
        }
    }

    /**
     * 核心更新函数，必须在OpMode的循环中持续调用。
     * 它负责处理所有RPM的计算和状态机的转换。
     */
    public void update() {
        // 只有在非空闲状态下才需要执行更新逻辑
        if (currentState == IntakeState.IDLE) {
            return;
        }

        // 1. 获取当前RPM并更新移动平均值
        double currentRPM = (intakeMotor.getVelocity() / MOTOR_TICKS_PER_REV) * 60;
        rpmHistory.add(currentRPM);
        if (rpmHistory.size() > MOVING_AVERAGE_WINDOW_SIZE) {
            rpmHistory.poll(); // 移除最旧的数据
        }

        double sum = 0;
        for (double rpm : rpmHistory) {
            sum += rpm;
        }
        if (!rpmHistory.isEmpty()) {
            averageRpm = sum / rpmHistory.size();
        }

        // 2. 根据当前状态执行逻辑
        switch (currentState) {
            case RUNNING:
                // 如果RPM足够高，并且当前RPM明显低于移动平均值，则认为检测到了像素
                if (averageRpm > MINIMUM_RPM_FOR_DETECTION && currentRPM < averageRpm * DROP_DETECTION_FACTOR) {
                    pixelCount++;
                    baselineRpmBeforeDip = averageRpm; // 记录骤降前的RPM基线
                    cooldownTimer.reset(); // 重置冷却计时器
                    currentState = IntakeState.DIP_DETECTED; // 进入冷却状态
                }
                break;
            case DIP_DETECTED:
                // 如果RPM已恢复到基线的一定比例，并且冷却时间已过，则返回正常运行状态
                if (currentRPM > baselineRpmBeforeDip * RECOVERY_FACTOR && cooldownTimer.milliseconds() > COOLDOWN_DURATION_MS) {
                    currentState = IntakeState.RUNNING;
                }
                break;
        }
    }

    // --- 获取器（Getters） ---

    public int getPixelCount() {
        return pixelCount;
    }

    public IntakeState getCurrentState() {
        return currentState;
    }

    public double getAverageRpm() {
        return averageRpm;
    }

    public double getCurrentRpm() {
        return (intakeMotor.getVelocity() / MOTOR_TICKS_PER_REV) * 60;
    }
}