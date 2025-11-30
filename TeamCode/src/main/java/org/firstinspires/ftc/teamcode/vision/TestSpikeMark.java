package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DecodeID", group = "Test")
public class TestSpikeMark extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotVision vision = new RobotVision();
        vision.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            vision.switchToAprilTagMode();

            int DecodeId = vision.getDecodeMarkId();

            telemetry.addData(">>> Decode ID", DecodeId == -1 ? "无" : DecodeId);

            if (DecodeId == 21) telemetry.addData("We need", "GPP");
            else if (DecodeId == 22) telemetry.addData("We need", "PGP");
            else if (DecodeId == 23) telemetry.addData("We need", "PPG");

            telemetry.update();
        }
        vision.close();
    }
}