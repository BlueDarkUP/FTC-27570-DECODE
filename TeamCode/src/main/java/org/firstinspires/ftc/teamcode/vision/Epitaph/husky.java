package org.firstinspires.ftc.teamcode.vision.Epitaph;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class husky {

    private HuskyLens huskyLens;
    private Telemetry telemetry;

    public husky(HardwareMap hardwareMap, Telemetry telemetry) {
        this.huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        this.telemetry = telemetry;
    }

    public boolean initialize() {
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
            return false;
        } else {
            telemetry.addData(">>", "HuskyLens connected successfully");
            return true;
        }
    }

    public void selectAlgorithm(HuskyLens.Algorithm algorithm) {
        huskyLens.selectAlgorithm(algorithm);
        telemetry.addData("Algorithm Selected", algorithm.toString());
    }

    public HuskyLens.Block[] getBlocks() {
        return huskyLens.blocks();
    }

    public void displayBlocksInfo() {
        HuskyLens.Block[] blocks = getBlocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block " + (i + 1), blocks[i].toString());
        }
    }
}