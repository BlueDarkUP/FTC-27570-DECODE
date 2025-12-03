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
    private DcMotor intakeMotor;
    private DcMotor mozartMotor;
    private CRServo holdServo;
    private CRServo classifyServo;
    private CRServo washerServo;
    private Servo bbbServo;

    // Sensors
    private NormalizedColorSensor ballSensor;
    private NormalizedColorSensor bufferSensor;

    // External Dependencies
    private ColorClassifier colorClassifier; // Updated dependency
    private Telemetry telemetry;
    private BooleanSupplier opModeActiveCheck;

    // Strategy Table
    private Map<String, List<Runnable>> strategyTable;

    private static final double BBB_INTAKE_POS = 0.92;
    private static final double BBB_SWAP_POS   = 0.35;

    private static final double POWER_FULL = 1.0;
    private static final double POWER_REVERSE_FULL = -1.0;
    private static final double POWER_REVERSE_HALF = -0.5;
    private static final double POWER_STOP = 0.0;

    // =========================================================
    // Constructor
    // =========================================================
    /**
     * @param classifier Pass in the already initialized ColorClassifier instance from your OpMode
     */
    public SortingSubsystem(HardwareMap hardwareMap, Telemetry telemetry, ColorClassifier classifier, BooleanSupplier activeCheck) {
        this.telemetry = telemetry;
        this.colorClassifier = classifier;
        this.opModeActiveCheck = activeCheck;

        // 1. Hardware Initialization
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        mozartMotor = hardwareMap.get(DcMotor.class, "MOZART");

        // Set Zero Power Behavior
        mozartMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        holdServo = hardwareMap.get(CRServo.class, "Hold");
        classifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        washerServo = hardwareMap.get(CRServo.class, "washer");
        bbbServo = hardwareMap.get(Servo.class, "bbb");

        ballSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        bufferSensor = hardwareMap.get(NormalizedColorSensor.class, "color2");

        // 2. Initialize Strategy Table
        initStrategyTable();
    }

    // =========================================================
    // Basic Control Functions
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
    // Core Action Implementations
    // =========================================================

    /**
     * Action 0: Pass / Final Intake
     */
    public void actPass() {
        telemetry.addData("Action", "Pass / Final Intake");
        telemetry.update();

        // Optional: Disable vision to save resources during pure mechanical pass
        colorClassifier.setEnabled(false);

        setIntakeMode();

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
     * Action 1: Extract
     */
    public void actExtract(ColorClassifier.DetectionResult targetColor) {
        telemetry.addData("Action", "Extracting " + targetColor);
        telemetry.update();

        // Enable Vision Pipeline
        colorClassifier.setEnabled(true);

        setIntakeMode();

        boolean extractionSuccess = false;

        while (opModeActiveCheck.getAsBoolean() && !extractionSuccess) {
            // Get Result from new Class
            ColorClassifier.DetectionResult detected = colorClassifier.getResult();

            // 1. Vision finds target
            if (detected == targetColor) {
                telemetry.addData("Vision", "Found Target! Reversing ClassifyServo");
                telemetry.update();

                // Reverse to extract
                classifyServo.setPower(POWER_REVERSE_FULL);

                // 2. Wait for color2 to confirm extraction
                while (opModeActiveCheck.getAsBoolean()) {
                    if (isBallInBuffer()) {
                        telemetry.addData("Sensor", "Color2 Detected -> Restore ClassifyServo");
                        telemetry.update();

                        // Restore and mark complete
                        classifyServo.setPower(POWER_FULL);
                        extractionSuccess = true;
                        break;
                    }
                }
            }
        }
    }

    /**
     * Action 3: Insert End
     */
    public void actInsertEnd() {
        telemetry.addData("Action", "Insert End");
        telemetry.update();

        setIntakeMode();
        while (opModeActiveCheck.getAsBoolean()) {
            if (isBallAtGate()) {
                mozartMotor.setPower(POWER_STOP); // Brake
                break;
            }
        }

        telemetry.addData("Info", "Braked. Waiting 1.5s...");
        telemetry.update();
        sleep(1500);

        telemetry.addData("Info", "Inserting...");
        telemetry.update();

        intakeMotor.setPower(POWER_REVERSE_HALF);
        holdServo.setPower(POWER_REVERSE_FULL);
        classifyServo.setPower(POWER_FULL);

        sleep(1500);

        holdServo.setPower(POWER_FULL);
        intakeMotor.setPower(POWER_STOP);
    }

    /**
     * Action 2: Insert Mid
     */
    public void actInsertMid(ColorClassifier.DetectionResult precedingColor) {
        telemetry.addData("Action", "Insert Mid after " + precedingColor);
        telemetry.update();

        // Enable Vision
        colorClassifier.setEnabled(true);
        setIntakeMode();

        boolean insertionDone = false;

        while (opModeActiveCheck.getAsBoolean() && !insertionDone) {
            ColorClassifier.DetectionResult detected = colorClassifier.getResult();

            // 1. Found preceding ball
            if (detected == precedingColor) {
                telemetry.addData("Trigger", "Saw " + precedingColor + " -> Inserting");
                telemetry.update();

                classifyServo.setPower(POWER_FULL);
                intakeMotor.setPower(POWER_STOP);
                holdServo.setPower(POWER_REVERSE_FULL);

                // 2. Wait for color2 to clear
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

        if (insertionDone) {
            holdServo.setPower(POWER_FULL);
            actPass();
        }
    }

    /**
     * Action 4: Swap
     */
    public void actSwap(ColorClassifier.DetectionResult expectInput) {
        telemetry.addData("Action", "Swapping... Expecting " + expectInput);
        telemetry.update();

        // Enable Vision
        colorClassifier.setEnabled(true);
        setIntakeMode();

        boolean swapDone = false;

        while (opModeActiveCheck.getAsBoolean() && !swapDone) {
            ColorClassifier.DetectionResult detected = colorClassifier.getResult();

            // 1. Found expected input
            if (detected == expectInput) {
                telemetry.addData("Trigger", "Saw " + expectInput + " -> Executing Swap");
                telemetry.update();

                // A. Open gate
                bbbServo.setPosition(BBB_SWAP_POS);

                // B. Push old, Suck new
                holdServo.setPower(POWER_FULL);
                classifyServo.setPower(POWER_REVERSE_FULL);

                // C. Delay
                sleep(478);

                // D. Wait for new ball in buffer
                while (opModeActiveCheck.getAsBoolean()) {
                    if (isBallInBuffer()) {
                        telemetry.addData("Sensor", "Color2 Detected New Ball -> Swap Success");
                        telemetry.update();

                        // --- Cleanup ---
                        bbbServo.setPosition(BBB_INTAKE_POS);
                        classifyServo.setPower(POWER_FULL);

                        swapDone = true;
                        break;
                    }
                }
            }
        }
    }

    // =========================================================
    // Strategy Table
    // =========================================================
    private void initStrategyTable() {
        strategyTable = new HashMap<>();

        // Use the Enum from ColorClassifier
        ColorClassifier.DetectionResult G = ColorClassifier.DetectionResult.GREEN;
        ColorClassifier.DetectionResult P = ColorClassifier.DetectionResult.PURPLE;

        // --- GGG Group ---
        strategyTable.put("GGG_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GGG_PGP", Arrays.asList(this::actPass));
        strategyTable.put("GGG_PPG", Arrays.asList(this::actPass));

        // --- GGP Group ---
        strategyTable.put("GGP_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GGP_PGP", Arrays.asList(this::actPass));
        strategyTable.put("GGP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        // --- GPG Group ---
        strategyTable.put("GPG_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GPG_PGP", Arrays.asList(
                () -> actExtract(G),
                () -> actInsertMid(P)
        ));
        strategyTable.put("GPG_PPG", Arrays.asList(this::actPass));

        // --- GPP Group ---
        strategyTable.put("GPP_GPP", Arrays.asList(this::actPass));
        strategyTable.put("GPP_PGP", Arrays.asList(
                () -> actExtract(G),
                () -> actInsertMid(P)
        ));
        strategyTable.put("GPP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        // --- PGG Group ---
        strategyTable.put("PGG_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actInsertMid(G)
        ));
        strategyTable.put("PGG_PGP", Arrays.asList(this::actPass));
        strategyTable.put("PGG_PPG", Arrays.asList(this::actPass));

        // --- PGP Group ---
        strategyTable.put("PGP_GPP", Arrays.asList(
                () -> actExtract(P),
                () -> actInsertMid(G)
        ));
        strategyTable.put("PGP_PGP", Arrays.asList(this::actPass));
        strategyTable.put("PGP_PPG", Arrays.asList(
                () -> actExtract(G),
                this::actInsertEnd
        ));

        // --- PPG Group ---
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

        // --- PPP Group ---
        strategyTable.put("PPP_GPP", Arrays.asList(this::actPass));
        strategyTable.put("PPP_PGP", Arrays.asList(this::actPass));
        strategyTable.put("PPP_PPG", Arrays.asList(this::actPass));
    }

    /**
     * Execute Strategy
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
                sleep(200);
            }
        } else {
            telemetry.addData("Warning", "No logic for " + key + ", defaulting to Pass");
            telemetry.update();
            actPass();
        }

        // Optional: Disable camera after strategy is done
        colorClassifier.setEnabled(false);
    }
}