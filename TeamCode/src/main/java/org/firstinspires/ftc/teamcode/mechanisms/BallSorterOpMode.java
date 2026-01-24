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

        // 提示初始化完成
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- 初始化循环 (Init Loop) ---
        while (!isStarted() && !isStopRequested()) {
            // 1. 处理按键输入
            if (gamepad1.dpad_left)  selectedInput = "GPP";
            if (gamepad1.dpad_up)    selectedInput = "PGP";
            if (gamepad1.dpad_right) selectedInput = "PPG";

            if (gamepad1.right_bumper) {
                targetIndex = (targetIndex + 1) % targetOptions.length;
                selectedTarget = targetOptions[targetIndex];
                sleep(250); // 防止按一次切换太快
            }

            // 2. [新增] 将当前选择显示在屏幕上
            telemetry.addData("--------------------------------", "");
            telemetry.addData(">> Selected Input (Start)", selectedInput);
            telemetry.addData(">> Selected Target (End)", selectedTarget);
            telemetry.addData("--------------------------------", "");
            telemetry.addData("Help", "D-Pad: Input | RB: Target");

            // 3. [重要] 必须调用 update() 才能刷新屏幕显示
            telemetry.update();
        }

        if (opModeIsActive()) {
            classifier.setEnabled(true);

            // 运行阶段也建议保留显示，让你知道最终确定的策略是什么
            telemetry.addData("Status", "Running Strategy...");
            telemetry.addData("Final Strategy", selectedInput + " -> " + selectedTarget);
            telemetry.update();

            sorter.executeStrategy(selectedInput, selectedTarget);

            telemetry.addData("Status", "Complete");
            telemetry.update();
            sleep(2000);
        }

        classifier.close();
    }
}