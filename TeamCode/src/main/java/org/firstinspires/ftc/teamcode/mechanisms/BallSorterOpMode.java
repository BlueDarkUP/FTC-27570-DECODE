package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.vision.Class.ColorClassifier;

@Autonomous(name = "Final Ball Sorter", group = "Competition")
public class BallSorterOpMode extends LinearOpMode {

    private String selectedInput = "GPP";
    private String selectedTarget = "GPP";

    private final String[] targetOptions = {"GPP", "PGP", "PPG"};
    private int targetIndex = 0;

    @Override
    public void runOpMode() {
        ColorClassifier classifier = new ColorClassifier();
        classifier.init(hardwareMap, "ClassifyCam", telemetry);

        SortingSubsystem sorter = new SortingSubsystem(hardwareMap, telemetry, classifier, this::opModeIsActive);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left)  selectedInput = "GPP";
            if (gamepad1.dpad_up)    selectedInput = "PGP";
            if (gamepad1.dpad_right) selectedInput = "PPG";

            if (gamepad1.right_bumper) {
                targetIndex = (targetIndex + 1) % targetOptions.length;
                selectedTarget = targetOptions[targetIndex];
                sleep(250);
            }
        }

        if (opModeIsActive()) {
            classifier.setEnabled(true);

            telemetry.addData("Status", "Running Strategy...");
            telemetry.update();

            sorter.executeStrategy(selectedInput, selectedTarget);

            telemetry.addData("Status", "Complete");
            telemetry.update();
            sleep(2000);
        }

        classifier.close();
    }
}