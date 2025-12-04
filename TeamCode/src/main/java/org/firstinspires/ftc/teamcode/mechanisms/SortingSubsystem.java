package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.Class.ColorClassifier;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class SortingSubsystem {

    // =========================================================
    // 硬件定义
    // =========================================================
    private DcMotor intakeMotor;
    private DcMotor mozartMotor;
    private CRServo holdServo;
    private CRServo classifyServo;
    private CRServo washerServo;
    private Servo bbbServo;

    private NormalizedColorSensor ballSensor;   // "color" (闸门处)
    private NormalizedColorSensor bufferSensor; // "color2" (缓存处)

    private ColorClassifier colorClassifier;
    private Telemetry telemetry;
    private BooleanSupplier opModeActiveCheck;

    private Map<String, List<Runnable>> strategyTable;

    // =========================================================
    // 常量定义
    // =========================================================
    private static final double BBB_INTAKE_POS = 0.92; // 关门
    private static final double BBB_SWAP_POS   = 0.35; // 吐球

    private static final double POWER_FULL = 1.0;
    private static final double POWER_REVERSE_FULL = -1.0;
    private static final double POWER_REVERSE_HALF = -0.5;
    private static final double POWER_STOP = 0.0;

    // =========================================================
    // 构造函数
    // =========================================================
    public SortingSubsystem(HardwareMap hardwareMap, Telemetry telemetry, ColorClassifier classifier, BooleanSupplier activeCheck) {
        this.telemetry = telemetry;
        this.colorClassifier = classifier;
        this.opModeActiveCheck = activeCheck;

        // 硬件初始化
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        mozartMotor = hardwareMap.get(DcMotor.class, "MOZART");

        mozartMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        holdServo = hardwareMap.get(CRServo.class, "Hold");
        classifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        washerServo = hardwareMap.get(CRServo.class, "washer");
        bbbServo = hardwareMap.get(Servo.class, "bbb");

        ballSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        bufferSensor = hardwareMap.get(NormalizedColorSensor.class, "color2");

        initStrategyTable();
    }

    // =========================================================
    // 基础控制
    // =========================================================
    public void setIntakeMode() {
        intakeMotor.setPower(POWER_FULL);
        mozartMotor.setPower(POWER_FULL);
        holdServo.setPower(POWER_FULL);
        classifyServo.setPower(POWER_FULL);
        washerServo.setPower(POWER_FULL);
        bbbServo.setPosition(BBB_INTAKE_POS);
    }

    private boolean isBallAtGate() {
        NormalizedRGBA colors = ballSensor.getNormalizedColors();
        return (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1);
    }

    private boolean isBallInBuffer() {
        NormalizedRGBA colors = bufferSensor.getNormalizedColors();
        return (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1);
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    // =========================================================
    // 核心动作函数 (Action Implementations)
    // =========================================================

    /** 动作 0: 统一发射 */
    public void actPass() {
        telemetry.addData("Action", "Pass / Final Intake");
        telemetry.update();
        setIntakeMode();
        while (opModeActiveCheck.getAsBoolean()) {
            if (isBallAtGate()) {
                mozartMotor.setPower(POWER_STOP);
                break;
            }
        }
    }

    /** 动作 1: 抽取 */
    public void actExtract(ColorClassifier.DetectionResult targetColor) {
        telemetry.addData("Action", "Extracting " + targetColor);
        telemetry.update();

        colorClassifier.setEnabled(true);
        setIntakeMode();

        boolean success = false;
        while (opModeActiveCheck.getAsBoolean() && !success) {
            if (colorClassifier.getResult() == targetColor) {
                // 1. 视觉确认，开始动作
                classifyServo.setPower(POWER_REVERSE_FULL);

                // [新增] 给舵机反转和接触球一点时间
                sleep(300);

                // 2. 等待球进入缓存
                while (opModeActiveCheck.getAsBoolean()) {
                    if (isBallInBuffer()) {
                        // [新增] 传感器虽然触发了，但球可能还在运动中，给一点时间让球停稳
                        sleep(300);

                        // 3. 恢复正转
                        classifyServo.setPower(POWER_FULL);

                        // [新增] 给舵机恢复正转一点时间，防止立刻进行下一个动作
                        sleep(200);

                        success = true;
                        break;
                    }
                }
            }
        }
    }

    /** 动作 3: 插队尾 */
    public void actInsertEnd() {
        telemetry.addData("Action", "Insert End");
        telemetry.update();

        // 吸入剩余球直到闸门
        setIntakeMode();
        while (opModeActiveCheck.getAsBoolean()) {
            if (isBallAtGate()) {
                mozartMotor.setPower(POWER_STOP);
                break;
            }
        }

        // 等待 3 秒
        sleep(3000);

        // 执行插入
        intakeMotor.setPower(POWER_REVERSE_HALF);
        holdServo.setPower(POWER_REVERSE_FULL);

        // [调整] 保持 1.5 秒，足够球进入
        sleep(1500);

        // 恢复状态
        holdServo.setPower(POWER_FULL);
        intakeMotor.setPower(POWER_STOP);

        // [新增] 缓冲时间
        sleep(200);
    }

    /** 动作 2: 插队中 */
    public void actInsertMid(ColorClassifier.DetectionResult precedingColor) {
        telemetry.addData("Action", "Insert Mid after " + precedingColor);
        telemetry.update();

        colorClassifier.setEnabled(true);
        setIntakeMode();

        boolean done = false;
        while (opModeActiveCheck.getAsBoolean() && !done) {
            if (colorClassifier.getResult() == precedingColor) {
                // 1. 发现前置球，执行动作
                classifyServo.setPower(POWER_FULL);
                intakeMotor.setPower(POWER_STOP);       // Brake
                holdServo.setPower(POWER_REVERSE_FULL); // Reverse

                // [新增] 给予机械臂动作时间
                sleep(200);

                // 2. 等待球离开缓存
                while (opModeActiveCheck.getAsBoolean()) {
                    if (!isBallInBuffer()) {
                        // 球已经不在缓存传感器视野内了

                        // [新增] 关键延时！球离开了传感器，但还没完全落入轨道中间。
                        // 必须给足时间让它掉下去，否则恢复Intake可能会打飞它。
                        sleep(500);

                        done = true;
                        break;
                    }
                }
            }
        }
        if (done) {
            holdServo.setPower(POWER_FULL);
            // [新增] 恢复 Hold 后给一点时间归位
            sleep(200);
            actPass();
        }
    }

    /** 动作 4: 置换 (Swap) */
    public void actSwap(ColorClassifier.DetectionResult expectInput) {
        telemetry.addData("Action", "Swapping... Expecting " + expectInput);
        telemetry.update();

        colorClassifier.setEnabled(true);
        setIntakeMode();

        boolean done = false;
        while (opModeActiveCheck.getAsBoolean() && !done) {
            if (colorClassifier.getResult() == expectInput) {
                // 1. 打开吐球口，开始反转
                bbbServo.setPosition(BBB_SWAP_POS);
                holdServo.setPower(POWER_FULL);
                classifyServo.setPower(POWER_REVERSE_FULL);

                // [新增] 关键延时！让机械结构完全打开，让球有时间开始运动
                // 防止还没张开嘴，球就撞上去了
                sleep(500);

                // 2. 等待新球进入
                while (opModeActiveCheck.getAsBoolean()) {
                    if (isBallInBuffer()) {
                        bbbServo.setPosition(BBB_INTAKE_POS); // 关门

                        sleep(200);

                        classifyServo.setPower(POWER_FULL);

                        done = true;
                        break;
                    }
                }
            }
        }
    }

    private void initStrategyTable() {
        strategyTable = new HashMap<>();

        ColorClassifier.DetectionResult G = ColorClassifier.DetectionResult.GREEN;
        ColorClassifier.DetectionResult P = ColorClassifier.DetectionResult.PURPLE;

        strategyTable.put("GPP_GPP", Arrays.asList(this::actPass));

        strategyTable.put("GPP_PGP", Arrays.asList(
                () -> actExtract(G),
                () -> actInsertMid(P)
        ));

        strategyTable.put("GPP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        strategyTable.put("PGP_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actInsertMid(G)
        ));

        strategyTable.put("PGP_PGP", Arrays.asList(this::actPass));

        strategyTable.put("PGP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));
        strategyTable.put("PPG_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actSwap(P),
                () -> actInsertMid(G)
        ));
        strategyTable.put("PPG_PGP", Arrays.asList(
                () -> actExtract(P),
                this::actInsertEnd
        ));
        strategyTable.put("PPG_PPG", Arrays.asList(this::actPass));
    }

    public void executeStrategy(String input, String target) {
        String key = input + "_" + target;
        telemetry.addData("Strategy", "Starting " + key);
        telemetry.update();

        List<Runnable> actions = strategyTable.get(key);
        if (actions != null && !actions.isEmpty()) {
            for (Runnable action : actions) {
                if (!opModeActiveCheck.getAsBoolean()) return;
                action.run();
                sleep(500);
            }
        } else {
            telemetry.addData("Error", "Invalid Pattern! Defaulting to Pass.");
            telemetry.update();
            actPass();
        }
    }
}