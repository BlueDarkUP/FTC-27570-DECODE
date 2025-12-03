package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.Class.ColorClassifier;

@Autonomous(name = "Final Ball Sorter", group = "Competition")
public class BallSorterOpMode extends LinearOpMode {

    private String selectedInput = "GGG";
    private String selectedTarget = "GPP";

    // Target array for cycling options
    private final String[] targetOptions = {"GPP", "PGP", "PPG"};
    private int targetIndex = 0;

    @Override
    public void runOpMode() {
        // 1. Initialize Vision (ColorClassifier)
        ColorClassifier colorClassifier = new ColorClassifier();
        // Make sure "ClassifyCam" matches the name in your Driver Station Configuration
        colorClassifier.init(hardwareMap, "ClassifyCam", telemetry);

        // 2. Initialize Subsystem
        // Pass the initialized colorClassifier and the opMode active check
        SortingSubsystem sorter = new SortingSubsystem(hardwareMap, telemetry, colorClassifier, this::opModeIsActive);

        // 3. UI Setup Loop (Init Loop)
        while (!isStarted() && !isStopRequested()) {
            // --- Input Selection (D-Pad & Buttons) ---
            if (gamepad1.dpad_down)  selectedInput = "GGG";
            if (gamepad1.dpad_up)    selectedInput = "GGP";
            if (gamepad1.dpad_left)  selectedInput = "GPG";
            if (gamepad1.dpad_right) selectedInput = "GPP";
            if (gamepad1.a)          selectedInput = "PGG";
            if (gamepad1.b)          selectedInput = "PGP";
            if (gamepad1.x)          selectedInput = "PPG";
            if (gamepad1.y)          selectedInput = "PPP";

            // --- Target Switching (Right Bumper) ---
            if (gamepad1.right_bumper) {
                targetIndex = (targetIndex + 1) % targetOptions.length;
                selectedTarget = targetOptions[targetIndex];
                // Simple debounce delay
                sleep(250);
            }

            // --- Display Current Status ---
            telemetry.addData("Config Mode", "Select Pattern");
            telemetry.addLine("-----------------");
            telemetry.addData("INPUT (D-Pad/Btn)", selectedInput);
            telemetry.addData("TARGET (RBumper)", selectedTarget);
            telemetry.addLine("-----------------");
            // Show live camera analysis during init to ensure camera is working
            telemetry.addData("Vision Preview", colorClassifier.getDebugInfo());
            telemetry.update();
        }

        // 4. Match Start
        if (opModeIsActive()) {
            // Note: We don't need to manually switch modes here anymore.
            // The SortingSubsystem will automatically enable the pipeline
            // when it performs an action that requires vision (Extract/Swap/InsertMid).

            telemetry.addData("Status", "Running Strategy...");
            telemetry.update();

            // Execute Core Logic
            sorter.executeStrategy(selectedInput, selectedTarget);

            // Task Complete
            telemetry.addData("Status", "Complete");
            telemetry.update();
            sleep(2000);
        }

        // Close Vision Resources
        colorClassifier.close();
    }
}