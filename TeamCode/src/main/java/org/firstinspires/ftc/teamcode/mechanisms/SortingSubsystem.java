package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.Class.ColorBlobProcessor;
import org.firstinspires.ftc.teamcode.vision.RobotVision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class SortingSubsystem {
    private DcMotor intakeMotor;
    private DcMotor mozartMotor;
    private CRServo holdServo;
    private CRServo classifyServo;
    private CRServo washerServo;
    private Servo bbbServo;

    // 传感器
    private NormalizedColorSensor ballSensor;
    private NormalizedColorSensor bufferSensor;

    // 外部依赖
    private RobotVision robotVision;
    private Telemetry telemetry;
    private BooleanSupplier opModeActiveCheck;

    // 策略表
    private Map<String, List<Runnable>> strategyTable;

    private static final double BBB_INTAKE_POS = 0.92;
    private static final double BBB_SWAP_POS   = 0.35;

    private static final double POWER_FULL = 1.0;
    private static final double POWER_REVERSE_FULL = -1.0;
    private static final double POWER_REVERSE_HALF = -0.5;
    private static final double POWER_STOP = 0.0;

    // =========================================================
    // 构造函数
    // =========================================================
    public SortingSubsystem(HardwareMap hardwareMap, Telemetry telemetry, RobotVision vision, BooleanSupplier activeCheck) {
        this.telemetry = telemetry;
        this.robotVision = vision;
        this.opModeActiveCheck = activeCheck;

        // 1. 硬件初始化
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        mozartMotor = hardwareMap.get(DcMotor.class, "MOZART");

        // 设置电机零功率行为
        mozartMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // MOZART需要刹车
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Intake 默认滑行

        holdServo = hardwareMap.get(CRServo.class, "Hold");
        classifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        washerServo = hardwareMap.get(CRServo.class, "washer");
        bbbServo = hardwareMap.get(Servo.class, "bbb");

        ballSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        bufferSensor = hardwareMap.get(NormalizedColorSensor.class, "color2");

        // 2. 初始化策略表
        initStrategyTable();
    }

    // =========================================================
    // 基础控制函数
    // =========================================================

    /**
     * 设置默认 Intake 状态
     * Intake: 1, MOZART: 1, Hold: 1, Classify: 1, Washer: 1, bbb: 0.25
     */
    public void setIntakeMode() {
        intakeMotor.setPower(POWER_FULL);
        mozartMotor.setPower(POWER_FULL);

        holdServo.setPower(POWER_FULL);
        classifyServo.setPower(POWER_FULL);
        washerServo.setPower(POWER_FULL);

        bbbServo.setPosition(BBB_INTAKE_POS);
    }

    /**
     * 检查闸门传感器 (color) 是否有读数
     */
    private boolean isBallAtGate() {
        NormalizedRGBA colors = ballSensor.getNormalizedColors();
        return (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1);
    }

    /**
     * 检查缓存传感器 (color2) 是否有读数
     */
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

    /**
     * 动作 0: 统一发射 / 默认 Intake 收尾
     * 逻辑: Intake模式运行 -> 遇到球 -> MOZART刹车
     */
    public void actPass() {
        telemetry.addData("Action", "Pass / Final Intake");
        telemetry.update();

        setIntakeMode(); // 确保电机都在转

        while (opModeActiveCheck.getAsBoolean()) {
            if (isBallAtGate()) {
                mozartMotor.setPower(POWER_STOP); // Brake
                telemetry.addData("Info", "Ball at Gate -> MOZART Brake");
                telemetry.update();
                break;
            }
        }
    }

    /**
     * 动作 1: 抽取
     * 逻辑: 视觉识别 -> ClassifyServo反转 -> color2检测到球 -> ClassifyServo恢复正转
     */
    public void actExtract(ColorBlobProcessor.DetectionResult targetColor) {
        telemetry.addData("Action", "Extracting " + targetColor);
        telemetry.update();

        robotVision.switchToColorMode();
        setIntakeMode(); // 基础转动

        boolean extractionSuccess = false;

        while (opModeActiveCheck.getAsBoolean() && !extractionSuccess) {
            ColorBlobProcessor.DetectionResult detected = robotVision.getColorResult();

            // 1. 视觉发现目标球
            if (detected == targetColor) {
                telemetry.addData("Vision", "Found Target! Reversing ClassifyServo");
                telemetry.update();

                // 立刻反转抽取
                classifyServo.setPower(POWER_REVERSE_FULL);

                // 2. 等待 color2 确认抽取成功 (有读数)
                while (opModeActiveCheck.getAsBoolean()) {
                    if (isBallInBuffer()) {
                        telemetry.addData("Sensor", "Color2 Detected -> Restore ClassifyServo");
                        telemetry.update();

                        // 恢复正转，标记完成
                        classifyServo.setPower(POWER_FULL);
                        extractionSuccess = true;
                        break;
                    }
                }
            }
        }
    }

    /**
     * 动作 3: 插队尾
     * 逻辑: Intake直到color有球 -> MOZART刹车 -> 等3秒 -> Intake反转0.5 & Hold反转 -> 完成
     */
    public void actInsertEnd() {
        telemetry.addData("Action", "Insert End");
        telemetry.update();

        // 1. 先把剩下的球吸过来 (类似 actPass 的前半段)
        setIntakeMode();
        while (opModeActiveCheck.getAsBoolean()) {
            if (isBallAtGate()) {
                mozartMotor.setPower(POWER_STOP); // Brake
                break;
            }
        }

        // 2. 刹车后等待
        telemetry.addData("Info", "Braked. Waiting 3s...");
        telemetry.update();
        sleep(1500);

        // 3. 执行插队动作
        telemetry.addData("Info", "Inserting...");
        telemetry.update();

        intakeMotor.setPower(POWER_REVERSE_HALF); // Intake 反转 0.5
        holdServo.setPower(POWER_REVERSE_FULL);   // Hold 反转 -1
        classifyServo.setPower(POWER_FULL);

        // 持续 1.5 秒
        sleep(1500);

        // 恢复状态
        holdServo.setPower(POWER_FULL);
        intakeMotor.setPower(POWER_STOP); // 停止或由下一个动作接管
    }

    /**
     * 动作 2: 插队中
     * 逻辑: 视觉识别[precedingColor] -> ClassifyServo=1, Intake=0(Brake), Hold=-1 -> color2变空 -> 恢复收尾
     */
    public void actInsertMid(ColorBlobProcessor.DetectionResult precedingColor) {
        telemetry.addData("Action", "Insert Mid after " + precedingColor);
        telemetry.update();

        robotVision.switchToColorMode();
        setIntakeMode();

        boolean insertionDone = false;

        while (opModeActiveCheck.getAsBoolean() && !insertionDone) {
            ColorBlobProcessor.DetectionResult detected = robotVision.getColorResult();

            // 1. 发现前置球
            if (detected == precedingColor) {
                telemetry.addData("Trigger", "Saw " + precedingColor + " -> Inserting");
                telemetry.update();

                // 执行插中动作组合
                classifyServo.setPower(POWER_FULL);     // 保持 1
                intakeMotor.setPower(POWER_STOP);       // Intake Brake
                holdServo.setPower(POWER_REVERSE_FULL); // Hold 反转

                // 2. 等待 color2 读数变为 0 (球离开缓存)
                while (opModeActiveCheck.getAsBoolean()) {
                    if (!isBallInBuffer()) {
                        telemetry.addData("Sensor", "Color2 Cleared -> Insert Success");
                        telemetry.update();
                        insertionDone = true;
                        break;
                    }
                }
            }
        }

        // 3. 收尾：变回Intake模式，吸入最后一个球
        if (insertionDone) {
            holdServo.setPower(POWER_FULL);
            actPass();
        }
    }

    /**
     * 动作 4: 置换 (Swap)
     * 逻辑: 视觉触发 -> bbb=0.98(吐), Hold=1(推), Classify=-1(吸) -> color2有球 -> 恢复
     */
    public void actSwap(ColorBlobProcessor.DetectionResult expectInput) {
        telemetry.addData("Action", "Swapping... Expecting " + expectInput);
        telemetry.update();

        robotVision.switchToColorMode();
        setIntakeMode(); // 基础状态

        boolean swapDone = false;

        while (opModeActiveCheck.getAsBoolean() && !swapDone) {
            ColorBlobProcessor.DetectionResult detected = robotVision.getColorResult();

            // 1. 视觉发现期望的输入球
            if (detected == expectInput) {
                telemetry.addData("Trigger", "Saw " + expectInput + " -> Executing Swap");
                telemetry.update();

                // --- 核心置换动作 ---
                // A. 开启吐球通道
                bbbServo.setPosition(BBB_SWAP_POS); // 0.98

                // B. Hold正转(推旧球), Classify反转(吸新球)
                holdServo.setPower(POWER_FULL);
                classifyServo.setPower(POWER_REVERSE_FULL);

                // C. 短暂延时，避开传感器误读 (可选)
                sleep(478);

                // D. 等待新球进入缓存 (color2 有读数)
                while (opModeActiveCheck.getAsBoolean()) {
                    if (isBallInBuffer()) {
                        telemetry.addData("Sensor", "Color2 Detected New Ball -> Swap Success");
                        telemetry.update();

                        // --- 收尾 ---
                        // 1. 及时关闭闸门
                        bbbServo.setPosition(BBB_INTAKE_POS); // 0.25

                        // 2. 恢复 Classify 正转
                        classifyServo.setPower(POWER_FULL);

                        swapDone = true;
                        break;
                    }
                }
            }
        }
    }

    // =========================================================
    // 策略表初始化 (Strategy Table)
    // =========================================================
    private void initStrategyTable() {
        strategyTable = new HashMap<>();

        ColorBlobProcessor.DetectionResult G = ColorBlobProcessor.DetectionResult.GREEN;
        ColorBlobProcessor.DetectionResult P = ColorBlobProcessor.DetectionResult.PURPLE;

        // --- GGG 组 ---
        strategyTable.put("GGG_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GGG_PGP", Arrays.asList(this::actPass));
        strategyTable.put("GGG_PPG", Arrays.asList(this::actPass));

        // --- GGP 组 ---
        strategyTable.put("GGP_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GGP_PGP", Arrays.asList(this::actPass));
        strategyTable.put("GGP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        // --- GPG 组 ---
        strategyTable.put("GPG_GPP", Arrays.asList(this::actPass));
        // GPG -> PGP: 抽取G, 插在P后面
        strategyTable.put("GPG_PGP", Arrays.asList(
                () -> actExtract(G),
                () -> actInsertMid(P)
        ));
        strategyTable.put("GPG_PPG", Arrays.asList(this::actPass));

        // --- GPP 组 ---
        strategyTable.put("GPP_GPP", Arrays.asList(this::actPass));
        // GPP -> PGP: 抽取G, 插在P后面
        strategyTable.put("GPP_PGP", Arrays.asList(
                () -> actExtract(G),
                () -> actInsertMid(P)
        ));
        // GPP -> PPG: 抽取G, 插尾
        strategyTable.put("GPP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        // --- PGG 组 ---
        // PGG -> GPP: 抽取P, 插在G后面
        strategyTable.put("PGG_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actInsertMid(G)
        ));
        strategyTable.put("PGG_PGP", Arrays.asList(this::actPass));
        strategyTable.put("PGG_PPG", Arrays.asList(this::actPass));

        // --- PGP 组 ---
        // PGP -> GPP: 抽取P, 插在G后面
        strategyTable.put("PGP_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actInsertMid(G)
        ));
        strategyTable.put("PGP_PGP", Arrays.asList(this::actPass));
        // [修正] PGP -> PPG: 抽取P, 置换(抓G吐P), 插尾
        strategyTable.put("PGP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        // --- PPG 组 ---
        // [修正] PPG -> GPP: 抽取P, 置换(抓P吐P), 插中(G后面)
        strategyTable.put("PPG_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actSwap(P),
                () -> actInsertMid(G)
        ));
        // PPG -> PGP: 抽取P, 插尾
        strategyTable.put("PPG_PGP", Arrays.asList(
                () -> actExtract(P),
                this::actInsertEnd
        ));
        strategyTable.put("PPG_PPG", Arrays.asList(this::actPass));

        // --- PPP 组 ---
        strategyTable.put("PPP_GPP", Arrays.asList(this::actPass));
        strategyTable.put("PPP_PGP", Arrays.asList(this::actPass));
        strategyTable.put("PPP_PPG", Arrays.asList(this::actPass));
    }

    /**
     * 执行策略
     */
    public void executeStrategy(String input, String target) {
        String key = input + "_" + target;
        telemetry.addData("Strategy", "Starting " + key);
        telemetry.update();

        List<Runnable> actions = strategyTable.get(key);
        if (actions != null && !actions.isEmpty()) {
            for (Runnable action : actions) {
                if (!opModeActiveCheck.getAsBoolean()) return;
                action.run();
                sleep(200); // 动作间缓冲
            }
        } else {
            telemetry.addData("Warning", "No logic for " + key + ", defaulting to Pass");
            telemetry.update();
            actPass();
        }
    }
}