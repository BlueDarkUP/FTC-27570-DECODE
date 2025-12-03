package org.firstinspires.ftc.teamcode.auto.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "远点 不吸 红方", group = "PedroPathing")
public class Far0Red extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // 状态机变量
    private int pathState = 0;
    private boolean isMozartBraked = false;

    // 硬件定义
    private DcMotorEx SH, MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private Servo LP, RP;
    private RevColorSensorV3 color;

    // 路径对象
    private PathChain path1_Preload;
    private PathChain path5_Sip1;
    private PathChain path6_Sip2;
    private PathChain path7_Sip3;
    private PathChain path8_Sip4;
    private PathChain path9_Score;
    private PathChain path10_Park;

    // 常量
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double FAR_SHOT_RPM = 3100.0;

    // 起始姿态
    // X: 144 - 56.2 = 87.8
    // Heading: 180 - 270 = -90
    private final Pose startPose = new Pose(87.800, 8.080, Math.toRadians(-90));

    public void buildPaths() {
        // Path 1: 预载
        // End X: 144 - 57.8 = 86.2
        // Heading: 180 - (-69) = 249
        path1_Preload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(87.800, 8.080), new Pose(86.200, 14.250)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(249))
                .build();

        // Path 5: 嘬一口
        // End X: 144 - 11 = 133
        // Heading: 249 -> 180 - (-156) = 336
        path5_Sip1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(86.200, 14.250), new Pose(133.000, 17.220)))
                .setLinearHeadingInterpolation(Math.toRadians(249), Math.toRadians(336))
                .build();

        // Path 6: 嘬两口
        // Control X: 144 - 14.37 = 129.63
        // End X: 144 - 11.16 = 132.84
        // Heading: 336 -> 336
        path6_Sip2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(133.000, 17.220), new Pose(129.630, 14.250), new Pose(132.840, 10.930)))
                .setLinearHeadingInterpolation(Math.toRadians(336), Math.toRadians(336))
                .build();

        // Path 7: 还想嘬
        // Control X: 144 - 10.93 = 133.07
        // End X: 144 - 10.7 = 133.3
        // Heading: 336 -> 180 - (-106) = 286
        path7_Sip3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(132.840, 10.930), new Pose(133.070, 12.590), new Pose(133.300, 10.455)))
                .setLinearHeadingInterpolation(Math.toRadians(336), Math.toRadians(286))
                .build();

        // Path 8: 嘬三口
        // Control X: 144 - 8.2 = 135.8
        // End X: 144 - 8.55 = 135.45
        // Heading: 286 -> 180 - (-90) = 270 (or -90)
        path8_Sip4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(133.300, 10.455), new Pose(135.800, 13.400), new Pose(135.450, 8.910)))
                .setLinearHeadingInterpolation(Math.toRadians(286), Math.toRadians(270))
                .build();

        // Path 9: 舍了舍了 (回分)
        // Control X: 144 - 32.3 = 111.7
        // End X: 144 - 57.74 = 86.26
        // Heading: 270 -> 249
        path9_Score = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(135.450, 8.910), new Pose(111.700, 14.000), new Pose(86.260, 14.250)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(249))
                .build();

        // Path 10: 让我看看 (停车)
        // Control X: 144 - 28.28 = 115.72
        // End X: 144 - 21.38 = 122.62
        // Heading: 249 -> 180 - 180 = 0
        path10_Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(86.260, 14.250), new Pose(115.720, 13.540), new Pose(122.620, 9.980)))
                .setLinearHeadingInterpolation(Math.toRadians(249), Math.toRadians(0))
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        // 硬件映射
        SH = hardwareMap.get(DcMotorEx.class, "SH");
        MOZART = hardwareMap.get(DcMotorEx.class, "MOZART");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        RP = hardwareMap.get(Servo.class, "RP");
        LP = hardwareMap.get(Servo.class, "LP");
        washer = hardwareMap.get(CRServo.class, "washer");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        ClassifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        // 电机配置
        SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SH.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(250, 0.1, 30, 13));
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        MOZART.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 舵机初始化
        RP.setPosition(0.1);
        LP.setPosition(1.0);

        // Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Far0Red Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("SH RPM", getShooterRPM());
        telemetry.addData("Mozart Braked", isMozartBraked);
        telemetry.update();
    }

    // --- 状态机 (逻辑保持不变) ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path 1: Preload Approach
                follower.followPath(path1_Preload, true);
                SH.setVelocity(calculateTicks(FAR_SHOT_RPM));
                setPathState(1);
                break;

            case 1: // Wait for Path 1 -> Shoot Preload
                if (!follower.isBusy()) {
                    runShooterLogic(FAR_SHOT_RPM);
                    if (actionTimer.getElapsedTimeSeconds() > 4.3) {
                        stopShooting();
                        setPathState(2);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 2: // Start Intake Sequence (Path 5)
                isMozartBraked = false; // 重置防走火
                follower.followPath(path5_Sip1, false);
                setPathState(3);
                break;

            case 3: // Path 5 Running
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(4);
                break;

            case 4: // Path 6: Sip 2
                follower.followPath(path6_Sip2, false);
                setPathState(5);
                break;

            case 5: // Path 6 Running
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(6);
                break;

            case 6: // Path 7: Sip 3
                follower.followPath(path7_Sip3, false);
                setPathState(7);
                break;

            case 7: // Path 7 Running
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(8);
                break;

            case 8: // Path 8: Sip 4
                follower.followPath(path8_Sip4, true); // 最后一段 Hold
                setPathState(9);
                break;

            case 9: // Path 8 Running
                runIntakeLogic();
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            // --- 吸取结束，回分 ---

            case 10: // Path 9: Return to Score
                sleep(200); // 稍微缓冲一下
                stopIntake();
                follower.followPath(path9_Score, true);
                SH.setVelocity(calculateTicks(FAR_SHOT_RPM));
                setPathState(11);
                break;

            case 11: // Wait for Path 9 -> Shoot Cycle 1
                if (!follower.isBusy()) {
                    runShooterLogic(FAR_SHOT_RPM);
                    if (actionTimer.getElapsedTimeSeconds() > 4.3) {
                        stopShooting();
                        setPathState(12);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 12: // Path 10: Park
                follower.followPath(path10_Park, true);
                setPathState(13);
                break;

            case 13: // End
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    // --- 辅助方法 ---

    private double calculateTicks(double rpm) {
        return (rpm * TICKS_PER_REV * GEAR_RATIO) / 60.0;
    }

    private void runShooterLogic(double targetRPM) {
        SH.setVelocity(calculateTicks(targetRPM));
        double currentRPM = getShooterRPM();

        if (Math.abs(currentRPM - targetRPM) <= 50) {
            MOZART.setPower(1.0);
            Hold.setPower(1.0);
            ClassifyServo.setPower(1.0);
            washer.setPower(1.0);
        } else {
            MOZART.setPower(0);
        }
    }

    private void stopShooting() {
        SH.setPower(0);
        MOZART.setPower(0);
        Hold.setPower(0);
        ClassifyServo.setPower(0);
        washer.setPower(0);
    }

    private double getShooterRPM() {
        return (SH.getVelocity() * 60.0) / (TICKS_PER_REV * GEAR_RATIO);
    }

    private void runIntakeLogic() {
        Intake.setPower(1.0);
        washer.setPower(1.0);
        Hold.setPower(1.0);
        ClassifyServo.setPower(1.0);

        NormalizedRGBA colors = color.getNormalizedColors();
        if (!isMozartBraked) {
            if (colors.red * 255 > 1 || colors.green * 255 > 1 || colors.blue * 255 > 1) {
                isMozartBraked = true;
            }
        }

        if (isMozartBraked) {
            MOZART.setPower(0.0);
        } else {
            MOZART.setPower(1.0);
        }
    }

    private void stopIntake() {
        Intake.setPower(0);
        washer.setPower(0);
        Hold.setPower(0);
        ClassifyServo.setPower(0);
        MOZART.setPower(0);
    }
    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
}