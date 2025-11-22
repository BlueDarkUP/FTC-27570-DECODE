package org.firstinspires.ftc.teamcode.vision.Epitaph;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "HuskyLensVision", group = "Sensor")
public class HuskyVision extends LinearOpMode {

    private husky huskyVision;
    private final int READ_PERIOD = 1;

    @Override
    public void runOpMode() {
        huskyVision = new husky(hardwareMap, telemetry);

        if (!huskyVision.initialize()) {
            telemetry.addLine("Failed to connect to HuskyLens. Check connection and configuration.");
            telemetry.update();
            sleep(5000);
            return;
        }

        // 3. 选择要使用的算法
        huskyVision.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        waitForStart();

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            huskyVision.displayBlocksInfo();

            telemetry.update();
        }
    }
}