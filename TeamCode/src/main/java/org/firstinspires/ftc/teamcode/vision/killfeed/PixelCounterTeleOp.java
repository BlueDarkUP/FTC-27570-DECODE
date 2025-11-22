package org.firstinspires.ftc.teamcode.vision.killfeed;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake Counter Test", group = "Tests")
public class PixelCounterTeleOp extends LinearOpMode {

    private DcMotorEx intakeMotor = null;
    private DcMotor inhaleMotor = null;
    private PixelCounterIntake intakeSystem;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        inhaleMotor = hardwareMap.get(DcMotor.class, "InhaleMotor");

        intakeSystem = new PixelCounterIntake(intakeMotor, inhaleMotor);

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.cross) {
                intakeSystem.start();
            } else {
                intakeSystem.stop();
            }

            intakeSystem.update();

            telemetry.addData("--- Intake System ---", "");
            telemetry.addData("Press 'Cross' (X) to Run", "");
            telemetry.addData("Pixel Count", intakeSystem.getPixelCount());
            telemetry.addData("Intake State", intakeSystem.getCurrentState());
            telemetry.addData("Current RPM", "%.2f", intakeSystem.getCurrentRpm());
            telemetry.addData("Average RPM", "%.2f", intakeSystem.getAverageRpm());
            telemetry.update();
        }
    }
}