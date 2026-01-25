package org.firstinspires.ftc.teamcode.auto.Norm.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor; // 新增
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // 新增
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "远点 不吸 蓝方", group = "A")
public class Far0Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // 状态机变量
    private int pathState = 0;
    private boolean isMozartBraked = false;

    // 硬件定义
    private DcMotorEx SH, MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private Servo LP, RP;

    private DistanceSensor distanceSensor2;

    // 路径对象
    private PathChain path1_Preload;
    private PathChain path5_Sip1;
    private PathChain path6_Sip2;
    private PathChain path7_Sip3;
    private PathChain path9_Score;
    private PathChain path10_Park;

    // 常量
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double FAR_SHOT_RPM = 3380.0; // 与 Far1CO 保持一致

    // 起始姿态
    private final Pose startPose = new Pose(56.200, 8.080, Math.toRadians(270));

    public void buildPaths() {
        // Path 1: 预载
        path1_Preload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.200, 8.080), new Pose(57.800, 14.250)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-69))
                .build();

        // Path 5: 嘬一口 (跳过了 2,3,4，直接开始吸取序列)
        path5_Sip1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(57.800, 14.250), new Pose(11.000, 17.220)))
                .setLinearHeadingInterpolation(Math.toRadians(-69), Math.toRadians(-156))
                .build();

        // Path 6: 嘬两口
        path6_Sip2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11.000, 17.220), new Pose(14.370, 14.250), new Pose(11.160, 10.930)))
                .setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(-156))
                .build();

        // Path 7: 还想嘬
        path7_Sip3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11.160, 10.930), new Pose(16, 16), new Pose(11, 9)))
                .setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(-90))
                .build();

        path9_Score = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(10.7, 9), new Pose(32.300, 16.000), new Pose(57.800, 14.250)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-69))
                .build();

        // Path 10: 让我看看 (停车)
        path10_Park = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(57.740, 14.250), new Pose(28.280, 16), new Pose(21.380, 9.980)))
                .setLinearHeadingInterpolation(Math.toRadians(-69), Math.toRadians(180))
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

        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "juju2");

        // 电机配置
        SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SH.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(250, 0.1, 30, 13));
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        MOZART.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 舵机初始化 (Far模式)
        RP.setPosition(0.1);
        LP.setPosition(1.0);

        // Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addData("Status", "Far0CO Initialized");
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

        // 调试信息
        if (distanceSensor2 != null) {
            telemetry.addData("D2", "%.1f", distanceSensor2.getDistance(DistanceUnit.MM));
        }

        telemetry.update();
    }

    // --- 状态机 ---
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
                    if (actionTimer.getElapsedTimeSeconds() > 4.5) {
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
                follower.followPath(path7_Sip3, true );
                setPathState(7);
                break;

            case 7: // Path 7 Running
                runIntakeLogic();
                if (!follower.isBusy()) setPathState(8);
                break;

            case 8:
                setPathState(9);
                break;

            case 9: // Path 8 Running
                runIntakeLogic();
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            // --- 吸取结束，回分 ---

            case 10:
                stopIntake();
                follower.followPath(path9_Score, true);
                SH.setVelocity(calculateTicks(FAR_SHOT_RPM));
                setPathState(11);
                break;

            case 11: // Wait for Path 9 -> Shoot Cycle 1
                if (!follower.isBusy()) {
                    runShooterLogic(FAR_SHOT_RPM);
                    if (actionTimer.getElapsedTimeSeconds() > 4.5) {
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
            MOZART.setPower(1);
            Hold.setPower(-1.0);
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

        double dist2 = distanceSensor2.getDistance(DistanceUnit.MM);

        if (!isMozartBraked) {
            // 任意一个传感器距离小于 50mm 则判定为有球
            if (dist2 < 50) {
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