package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose; // 只需要导入 Pose
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
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

public class AutoSortingSubsystem {

    // 硬件
    private DcMotor intakeMotor, mozartMotor;
    private CRServo holdServo, classifyServo, washerServo;
    private Servo bbbServo;
    private NormalizedColorSensor ballSensor, bufferSensor;

    // 外部依赖
    private ColorClassifier colorClassifier;
    private Telemetry telemetry;
    private BooleanSupplier opModeActiveCheck;
    private Follower follower;

    private Map<String, List<Runnable>> strategyTable;

    // 常量
    private static final double BBB_INTAKE_POS = 0.92;
    private static final double BBB_SWAP_POS   = 0.35;
    private static final double INTAKE_TIMEOUT_SEC = 3.5;

    public AutoSortingSubsystem(HardwareMap hardwareMap, Telemetry telemetry, ColorClassifier classifier, BooleanSupplier activeCheck, Follower follower) {
        this.telemetry = telemetry;
        this.colorClassifier = classifier;
        this.opModeActiveCheck = activeCheck;
        this.follower = follower;

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

    // --- 核心工具方法 ---

    private void safeSleep(long ms) {
        Timer t = new Timer();
        t.resetTimer();
        while (opModeActiveCheck.getAsBoolean() && t.getElapsedTimeSeconds() * 1000 < ms) {
            follower.update();
        }
    }

    public void setIntakeMode() {
        intakeMotor.setPower(1.0);
        mozartMotor.setPower(1.0);
        holdServo.setPower(1.0);
        classifyServo.setPower(1.0);
        washerServo.setPower(1.0);
        bbbServo.setPosition(BBB_INTAKE_POS);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
        mozartMotor.setPower(0);
        holdServo.setPower(0);
        classifyServo.setPower(0);
        washerServo.setPower(0);
    }

    private boolean isBallAtGate() {
        NormalizedRGBA colors = ballSensor.getNormalizedColors();
        return (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1);
    }

    private boolean isBallInBuffer() {
        NormalizedRGBA colors = bufferSensor.getNormalizedColors();
        return (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1);
    }

    // --- 动作实现 ---

    public void actPass() {
        telemetry.addData("Action", "Pass");
        telemetry.update();
        setIntakeMode();

        Timer timeout = new Timer();
        timeout.resetTimer();

        while (opModeActiveCheck.getAsBoolean()) {
            follower.update();
            if (isBallAtGate()) break;
            if (timeout.getElapsedTimeSeconds() > INTAKE_TIMEOUT_SEC) break;
        }

        safeSleep(500);
        mozartMotor.setPower(0);
    }

    public void actExtract(ColorClassifier.DetectionResult targetColor) {
        telemetry.addData("Action", "Extracting " + targetColor);
        telemetry.update();
        colorClassifier.setEnabled(true);
        setIntakeMode();

        Timer timeout = new Timer();
        timeout.resetTimer();

        boolean success = false;
        while (opModeActiveCheck.getAsBoolean() && !success) {
            follower.update();
            if (timeout.getElapsedTimeSeconds() > INTAKE_TIMEOUT_SEC) break;

            if (colorClassifier.getResult() == targetColor) {
                classifyServo.setPower(-1.0);
                safeSleep(300);

                while (opModeActiveCheck.getAsBoolean()) {
                    follower.update();
                    if (isBallInBuffer()) {
                        safeSleep(300);
                        classifyServo.setPower(1.0);
                        safeSleep(200);
                        success = true;
                        break;
                    }
                    if (timeout.getElapsedTimeSeconds() > INTAKE_TIMEOUT_SEC) break;
                }
            }
        }
    }

    public void actInsertMid(ColorClassifier.DetectionResult precedingColor) {
        telemetry.addData("Action", "Insert Mid after " + precedingColor);
        telemetry.update();
        colorClassifier.setEnabled(true);
        setIntakeMode();

        Timer timeout = new Timer();
        timeout.resetTimer();

        while (opModeActiveCheck.getAsBoolean()) {
            follower.update();
            if (timeout.getElapsedTimeSeconds() > INTAKE_TIMEOUT_SEC) break;

            if (colorClassifier.getResult() == precedingColor) {
                classifyServo.setPower(1.0);
                intakeMotor.setPower(0);
                holdServo.setPower(-1.0);
                safeSleep(200);

                while (opModeActiveCheck.getAsBoolean()) {
                    follower.update();
                    if (!isBallInBuffer()) {
                        safeSleep(500);
                        break;
                    }
                    if (timeout.getElapsedTimeSeconds() > INTAKE_TIMEOUT_SEC) break;
                }
                holdServo.setPower(1.0);
                safeSleep(200);
                actPass();
                return;
            }
        }
    }

    public void actInsertEnd() {
        telemetry.addData("Action", "Insert End");
        telemetry.update();
        setIntakeMode();

        Timer timeout = new Timer();
        timeout.resetTimer();

        while (opModeActiveCheck.getAsBoolean()) {
            follower.update();
            if (isBallAtGate() || timeout.getElapsedTimeSeconds() > 2.0) {
                mozartMotor.setPower(0);
                break;
            }
        }

        safeSleep(500);
        intakeMotor.setPower(-0.5);
        holdServo.setPower(-1.0);
        safeSleep(1500);
        holdServo.setPower(1.0);
        intakeMotor.setPower(0);
        safeSleep(200);
    }

    public void actSwap(ColorClassifier.DetectionResult expectInput) {
        telemetry.addData("Action", "Swapping... Expecting " + expectInput);
        telemetry.update();
        colorClassifier.setEnabled(true);
        setIntakeMode();

        Timer timeout = new Timer();
        timeout.resetTimer();

        boolean done = false;
        while (opModeActiveCheck.getAsBoolean() && !done) {
            follower.update();
            if (timeout.getElapsedTimeSeconds() > INTAKE_TIMEOUT_SEC) break;

            if (colorClassifier.getResult() == expectInput) {
                performChassisMove(-6.0); // 后退

                bbbServo.setPosition(BBB_SWAP_POS);
                holdServo.setPower(1.0);
                classifyServo.setPower(-1.0);
                safeSleep(600);

                Timer swapTimer = new Timer();
                swapTimer.resetTimer();
                while (opModeActiveCheck.getAsBoolean()) {
                    follower.update();
                    if (isBallInBuffer()) {
                        bbbServo.setPosition(BBB_INTAKE_POS);
                        safeSleep(200);
                        classifyServo.setPower(1.0);
                        performChassisMove(6.0); // 前进
                        done = true;
                        break;
                    }
                    if (swapTimer.getElapsedTimeSeconds() > 2.0) {
                        bbbServo.setPosition(BBB_INTAKE_POS);
                        classifyServo.setPower(1.0);
                        performChassisMove(6.0); // 前进
                        done = true;
                        break;
                    }
                }
            }
        }
    }

    // --- 修复后的底盘移动方法 ---
    private void performChassisMove(double inches) {
        Pose current = follower.getPose();
        double heading = current.getHeading();

        // 计算目标 Pose
        double dx = inches * Math.cos(heading);
        double dy = inches * Math.sin(heading);
        Pose target = new Pose(current.getX() + dx, current.getY() + dy, heading);

        // 直接传递 Pose 对象给 BezierLine，不需要 new Point()
        // 因为 Pose 通常继承自 Point，或者 PedroPathing 允许使用 Pose 构建直线
        Path movePath = new Path(new BezierLine(current, target));
        movePath.setConstantHeadingInterpolation(heading);

        follower.followPath(movePath, true);

        Timer moveTimer = new Timer();
        moveTimer.resetTimer();
        while (opModeActiveCheck.getAsBoolean() && follower.isBusy()) {
            follower.update();
            if (moveTimer.getElapsedTimeSeconds() > 1.5) break;
        }
    }

    private void initStrategyTable() {
        strategyTable = new HashMap<>();
        ColorClassifier.DetectionResult G = ColorClassifier.DetectionResult.GREEN;
        ColorClassifier.DetectionResult P = ColorClassifier.DetectionResult.PURPLE;

        strategyTable.put("GPP_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GPP_PGP", Arrays.asList(() -> actExtract(G), () -> actInsertMid(P)));
        strategyTable.put("GPP_PPG", Arrays.asList(() -> actExtract(G), this::actInsertEnd));

        strategyTable.put("PGP_GPP", Arrays.asList(() -> actExtract(P), () -> actInsertMid(G)));
        strategyTable.put("PGP_PGP", Arrays.asList(this::actPass));
        strategyTable.put("PGP_PPG", Arrays.asList(() -> actExtract(G), this::actInsertEnd));

        strategyTable.put("PPG_GPP", Arrays.asList(() -> actExtract(P), () -> actSwap(P), () -> actInsertMid(G)));
        strategyTable.put("PPG_PGP", Arrays.asList(() -> actExtract(P), this::actInsertEnd));
        strategyTable.put("PPG_PPG", Arrays.asList(this::actPass));
    }

    public void executeStrategy(String input, String target) {
        String key = input + "_" + target;
        telemetry.addData("Strategy", "Executing " + key);
        telemetry.update();

        List<Runnable> actions = strategyTable.get(key);
        if (actions != null) {
            for (Runnable action : actions) {
                if (!opModeActiveCheck.getAsBoolean()) return;
                action.run();
                safeSleep(400);
            }
        } else {
            actPass();
        }
    }
}