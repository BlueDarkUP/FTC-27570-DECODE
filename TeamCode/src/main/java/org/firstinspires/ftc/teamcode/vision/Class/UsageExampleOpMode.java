package org.firstinspires.ftc.teamcode.vision.Class;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vision Test Interface", group = "Test")
public class UsageExampleOpMode extends LinearOpMode {

    // 1. 声明分类器对象
    ColorClassifier vision = new ColorClassifier();

    @Override
    public void runOpMode() {
        // 2. 初始化视觉 (传入 hardwareMap, 摄像头名字, 和 telemetry)
        vision.init(hardwareMap, "ClassifyCam", telemetry);

        telemetry.addData("Status", "Vision Initialized");
        telemetry.update();

        // 等待开始时的循环（可选，用于在比赛开始前确认摄像头是否工作）
        while (!isStarted()) {
            telemetry.addData("Preview Result", vision.getResult());
            telemetry.addData("Debug", vision.getDebugInfo());
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        while (opModeIsActive()) {
            // 3. 获取结果
            ColorClassifier.DetectionResult result = vision.getResult();

            // 4. 根据结果执行逻辑
            switch (result) {
                case GREEN:
                    telemetry.addData("Action", "LIFT UP (Green)");
                    // robot.lift.setPower(1.0);
                    break;
                case PURPLE:
                    telemetry.addData("Action", "DRIVE FWD (Purple)");
                    // robot.drive.moveForward();
                    break;
                case NONE:
                    telemetry.addData("Action", "STOP (None)");
                    // robot.stop();
                    break;
            }

            telemetry.addData("Raw Counts", vision.getDebugInfo());
            telemetry.update();
        }

        // 5. 结束时关闭 (可选，EasyOpenCv通常会自动处理)
        vision.close();
    }
}