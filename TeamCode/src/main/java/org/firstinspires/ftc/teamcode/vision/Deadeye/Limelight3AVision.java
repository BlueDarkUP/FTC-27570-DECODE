package org.firstinspires.ftc.teamcode.vision.Deadeye;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight3A Vision", group = "Sensor")
public class Limelight3AVision extends LinearOpMode {

    private Deadeye DeadeyeAPI;

    @Override
    public void runOpMode() throws InterruptedException {DeadeyeAPI = new Deadeye(hardwareMap);
        DeadeyeAPI.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.addData(">", "Press (A) to reset intake count."); // 新增提示
        telemetry.update();

        waitForStart();

        DeadeyeAPI.resetIntakeCount();

        while (opModeIsActive()) {
            DeadeyeAPI.update();

            if (gamepad1.a) {
                DeadeyeAPI.resetIntakeCount();
            }

            telemetry.addData("Successful Intakes", DeadeyeAPI.getSuccessfulIntakeCount());
            telemetry.addLine("------------------------------------");


            LLResultTypes.DetectorResult target = DeadeyeAPI.getTargetClosestToAnchor();

            if (target != null) {
                int targetId = target.getClassId();
                telemetry.addData("Closest to Anchor", "ID: %d (%s)",
                        targetId, "purple".equals(target.getClassName()) ? "Purple" : "Green");

                double[] errors = DeadeyeAPI.calculateAlignmentError(target);
                if (errors != null) {
                    double errorX = errors[0];
                    double errorY = errors[1];
                    telemetry.addData("Alignment Error", "X_Error: %.1f cm, Y_Error: %.1f cm", errorX, errorY);
                } else {
                    telemetry.addData("Alignment Error", "Cannot be calculated");
                }
            } else {
                telemetry.addData("Target", "No valid targets detected");
            }
            telemetry.update();
        }
        DeadeyeAPI.stop();
    }
}