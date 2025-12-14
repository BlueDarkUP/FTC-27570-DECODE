package org.firstinspires.ftc.teamcode.auto.cpx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "近点 15 蓝方", group = "PedroPathing")
public class CloseBlueCPX extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // 状态机变量
    private int pathState = 0;

    // 射速锁存状态
    private boolean hasReachedTargetSpeed = false;
    private boolean isMozartBraked = false;

    // 硬件定义
    private DcMotorEx SH, MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private Servo LP, RP;

    private DistanceSensor distanceSensor;
    private DistanceSensor distanceSensor2;

    // 路径定义
    private PathChain path1_Preload, path2_ToObelisk, path3_Intake1, path4_Score1, path5_ToSpike2, path6_Intake2, path7_Maneuver, path8_Score2, path9_ToSpike3, path10_Intake3, path11_Score3;
    private PathChain path12_ToSubmersible;
    private PathChain path13_SubmersibleIntake;
    private PathChain path14_ReturnFromSub; // 动态生成
    private PathChain path15_Park; // 动态生成

    // 常量配置
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    PathConstraints slowConstraints = new PathConstraints(30.0, 10.0, 1.0, 1.0);
    PathConstraints subIntakeConstraints = new PathConstraints(10.0, 5.0, 1.0, 1.0);

    private final Pose startPose = new Pose(33.600, 135.560, Math.toRadians(270));

    public void buildPaths() {
        // Path 1: Preload
        path1_Preload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(33.600, 135.560), new Pose(48.500, 95.900)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-48))
                .build();

        // Path 2: 观测
        path2_ToObelisk = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(48.500, 95.900), new Pose(60, 83.170), new Pose(43.400, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 3: 吸取 1
        path3_Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42.400, 84.000), new Pose(19, 83.644)))
                .setConstraints(slowConstraints)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: 吸完1 -> 直接去发射1
        path4_Score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18.000, 83.644), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 5: 射完1 -> 去吸2
        path5_ToSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.900, 84.000), new Pose(48.000, 63.5)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 6: 吸取 2
        path6_Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 63.5), new Pose(9.500, 63.5)))
                .setConstraints(slowConstraints)
                .setTangentHeadingInterpolation()
                .build();

        // Path 7: 吸完2 -> 去拉闸 (Maneuver) **微调后**
        path7_Maneuver = follower.pathBuilder()
                .setConstraints(slowConstraints)
                .addPath(new BezierCurve(new Pose(9.500, 63.5), new Pose(32.000, 69.000), new Pose(18.000, 73.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        // Path 8: 拉完闸 -> 去发射 2
        path8_Score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18.000, 73.000), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-48))
                .build();

        // Path 9: 准备吸第三排
        path9_ToSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.880, 84.000), new Pose(48.000, 38)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                .build();

        // Path 10: 吸取 3
        path10_Intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48.000, 38), new Pose(9.500, 38)))
                .setConstraints(slowConstraints)
                .setTangentHeadingInterpolation()
                .build();

        // Path 11: 发射 Cycle 3
        path11_Score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9.5, 38), new Pose(59.900, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48))
                .build();

        // Path 12: 去往潜水艇入口
        path12_ToSubmersible = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.900, 84.000), new Pose(12.150, 40.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(-130))
                .build();

        // Path 13: 潜水艇盲吸 (构建一条很长的线，靠时间停止)
        path13_SubmersibleIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.150, 55.000), new Pose(12.150, 20.000)))
                .setConstraints(subIntakeConstraints)
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(-130))
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        SH = hardwareMap.get(DcMotorEx.class, "SH");
        MOZART = hardwareMap.get(DcMotorEx.class, "MOZART");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        RP = hardwareMap.get(Servo.class, "RP");
        LP = hardwareMap.get(Servo.class, "LP");
        washer = hardwareMap.get(CRServo.class, "washer");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        ClassifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "juju");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "juju2");

        SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfOrig = new PIDFCoefficients(250, 0.1, 30, 13);
        SH.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfOrig);
        MOZART.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        LP.setPosition(0.3546);
        RP.setPosition(0.667);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Close3Blue Submersible Initialized");
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
        telemetry.addData("SH RPM", getShooterRPM());
        telemetry.addData("Reached Speed", hasReachedTargetSpeed);

        // 调试用：显示距离
        if (distanceSensor != null && distanceSensor2 != null) {
            telemetry.addData("Dist1", "%.1f", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Dist2", "%.1f", distanceSensor2.getDistance(DistanceUnit.MM));
        }

        telemetry.update();
    }

    // --- 状态机逻辑 ---
    public void autonomousPathUpdate() {
        switch (pathState) {
            // --- Standard 3 Cycles ---
            case 0:
                follower.followPath(path1_Preload, true);
                SH.setVelocity(calculateTicks(2350));
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (runLatchingShooterLogic(2350, 0.8)) {
                        stopShooting();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                follower.followPath(path2_ToObelisk, true);
                setPathState(3);
                break;
            case 3:
                if (!follower.isBusy()) setPathState(4);
                break;
            case 4:
                isMozartBraked = false;
                follower.setMaxPower(0.9);
                follower.followPath(path3_Intake1, false);
                setPathState(5);
                break;
            case 5:
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(6);
                break;
            case 6: // Score 1
                stopIntake();
                follower.setMaxPower(1);
                follower.followPath(path4_Score1, true);
                SH.setVelocity(calculateTicks(2550));
                setPathState(7);
                break;
            case 7: // Shoot 1
                if (!follower.isBusy()) {
                    if (runLatchingShooterLogic(2550, 0.85)) {
                        stopShooting();
                        setPathState(8);
                    }
                }
                break;
            case 8: // To Spike 2
                follower.followPath(path5_ToSpike2, true);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) setPathState(10);
                break;
            case 10: // Intake 2
                isMozartBraked = false;
                follower.setMaxPower(0.7);
                follower.followPath(path6_Intake2, false);
                setPathState(11);
                break;
            case 11:
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(12);
                break;
            case 12: // Maneuver
                stopIntake();
                follower.setMaxPower(1);
                follower.followPath(path7_Maneuver, false);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    LP.setPosition(0.8156);
                    RP.setPosition(0.262);
                    SH.setVelocity(calculateTicks(2300));
                    setPathState(14);
                }
                break;
            case 14: // Score 2
                follower.setMaxPower(1);
                follower.followPath(path8_Score2, true);
                setPathState(15);
                break;
            case 15: // Shoot 2
                if (!follower.isBusy()) {
                    if (runLatchingShooterLogic(2550, 0.85)) {
                        stopShooting();
                        setPathState(16);
                    }
                }
                break;
            case 16: // To Spike 3
                follower.followPath(path9_ToSpike3, true);
                setPathState(17);
                break;
            case 17:
                if (!follower.isBusy()) setPathState(18);
                break;
            case 18: // Intake 3
                isMozartBraked = false;
                follower.setMaxPower(0.7);
                follower.followPath(path10_Intake3, false);
                setPathState(19);
                break;
            case 19:
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(20);
                break;
            case 20: // Score 3
                stopIntake();
                follower.setMaxPower(1);
                follower.followPath(path11_Score3, true);
                SH.setVelocity(calculateTicks(2500));
                setPathState(21);
                break;
            case 21: // Shoot 3
                if (!follower.isBusy()) {
                    if (runLatchingShooterLogic(2500, 0.85)) {
                        stopShooting();
                        setPathState(22);
                    }
                }
                break;

            // --- Submersible Sequence ---
            case 22:
                follower.followPath(path12_ToSubmersible, false);
                setPathState(23);
                break;
            case 23:
                if(!follower.isBusy()) setPathState(24);
                break;
            case 24:
                isMozartBraked = false;
                follower.setMaxPower(0.7);
                follower.followPath(path13_SubmersibleIntake, false);
                actionTimer.resetTimer();
                setPathState(25);
                break;
            case 25:
                runIntakeLogic();
                if (actionTimer.getElapsedTimeSeconds() > 1.95) {
                    Pose currentPose = follower.getPose();
                    path14_ReturnFromSub = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, new Pose(59.900, 84.000)))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(-48))
                            .build();
                    stopIntake();
                    follower.setMaxPower(1);
                    follower.followPath(path14_ReturnFromSub, true);
                    SH.setVelocity(calculateTicks(2500));
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) setPathState(27);
                break;
            case 27:
                if (runLatchingShooterLogic(2500, 0.85)) {
                    stopShooting();
                    setPathState(28);
                }
                break;
            case 28:
                path15_Park = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(59.900, 84.000), new Pose(28.000, 70.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180))
                        .build();
                follower.followPath(path15_Park, true);
                setPathState(29);
                break;
            case 29:
                if(!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        hasReachedTargetSpeed = false;
    }

    // --- 功能函数 ---
    private boolean runLatchingShooterLogic(double targetRPM, double fireDuration) {
        SH.setVelocity(calculateTicks(targetRPM));
        Hold.setPower(-1);
        if (!hasReachedTargetSpeed) {
            if (Math.abs(getShooterRPM() - targetRPM) <= 50) {
                hasReachedTargetSpeed = true;
                actionTimer.resetTimer();
            }
        }
        if (hasReachedTargetSpeed) {
            MOZART.setPower(1.0); Hold.setPower(1.0); ClassifyServo.setPower(1.0); washer.setPower(1.0);
            if (actionTimer.getElapsedTimeSeconds() >= fireDuration) return true;
        } else {
            MOZART.setPower(0);
        }
        return false;
    }
    private double calculateTicks(double rpm) { return (rpm * TICKS_PER_REV * GEAR_RATIO) / 60.0; }
    private void stopShooting() { SH.setPower(0); MOZART.setPower(0); Hold.setPower(0); ClassifyServo.setPower(0); washer.setPower(0); }
    private double getShooterRPM() { return (SH.getVelocity() * 60.0) / (TICKS_PER_REV * GEAR_RATIO); }

    // --- 修改：使用距离传感器逻辑 ---
    private void runIntakeLogic() {
        Intake.setPower(1.0); washer.setPower(1.0); Hold.setPower(1.0); ClassifyServo.setPower(1.0);

        // 获取距离
        double dist1 = distanceSensor.getDistance(DistanceUnit.MM);
        double dist2 = distanceSensor2.getDistance(DistanceUnit.MM);

        if (!isMozartBraked) {
            if (dist1 < 50 || dist2 < 50) {
                isMozartBraked = true;
            }
        }
        MOZART.setPower(isMozartBraked ? 0.0 : 1.0);
    }

    private void stopIntake() { Intake.setPower(0); washer.setPower(0); Hold.setPower(0); ClassifyServo.setPower(0); MOZART.setPower(0); }
}