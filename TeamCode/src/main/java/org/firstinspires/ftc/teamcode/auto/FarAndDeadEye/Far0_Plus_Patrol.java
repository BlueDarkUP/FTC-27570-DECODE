package org.firstinspires.ftc.teamcode.auto.FarAndDeadEye;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "远点 巡逻 蓝方 (最终版 - 角度修正)", group = "PedroPathing")
public class Far0_Plus_Patrol extends OpMode {

    private int mainState = 0;

    private Follower follower;
    private Limelight3A limelight;
    private DcMotorEx SH, MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private Servo LP, RP;
    private DistanceSensor distanceSensor2;

    private Timer actionTimer, patrolTimer;
    private boolean isMozartBraked = false;
    private List<PathChain> patrolPaths = new ArrayList<>();
    private List<Pose> patrolEndPoses = new ArrayList<>();
    private int currentPatrolPathIndex = 0;
    private boolean isIntakePermanentlyOn = false;

    private PathChain dynamicPath;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double FAR_SHOT_RPM = 3380.0;
    private static final double CAMERA_HEIGHT_CM = 31.5;
    private static final double CAMERA_MOUNT_ANGLE_DEG = -53.5;
    private static final double CM_TO_INCH = 1.0 / 2.54;
    private static final double MAX_TARGET_DISTANCE_IN = 100.0;

    private final Pose startPose = new Pose(56.200, 8.080, Math.toRadians(270));
    private final Pose farScorePose = new Pose(57.800, 14.250, Math.toRadians(-69));
    private final Pose patrolStartPose = new Pose(20.000, 11.000, Math.toRadians(180));
    // 为停车点也定义一个明确的朝向
    private final Pose finalParkPose = new Pose(21.380, 9.980, Math.toRadians(180));

    private PathChain path1_Preload, path5_Sip1, path6_Sip2, path7_Sip3, path8_Score, path10_Transition;

    public void buildPaths() {
        path1_Preload = follower.pathBuilder().addPath(new BezierLine(startPose, farScorePose)).setLinearHeadingInterpolation(startPose.getHeading(), farScorePose.getHeading()).build();
        path5_Sip1 = follower.pathBuilder().addPath(new BezierLine(farScorePose, new Pose(11.000, 17.220))).setLinearHeadingInterpolation(Math.toRadians(-69), Math.toRadians(-156)).build();
        path6_Sip2 = follower.pathBuilder().addPath(new BezierCurve(new Pose(11.000, 17.220), new Pose(14.370, 14.250), new Pose(11.160, 10.930))).setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(-156)).build();
        path7_Sip3 = follower.pathBuilder().addPath(new BezierCurve(new Pose(11.160, 10.930), new Pose(16, 16), new Pose(11, 9))).setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(-90)).build();
        path8_Score = follower.pathBuilder().addPath(new BezierCurve(new Pose(10.7, 9), new Pose(32.300, 14.000), farScorePose)).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-69)).build();
        path10_Transition = follower.pathBuilder().addPath(new BezierLine(farScorePose, patrolStartPose)).setLinearHeadingInterpolation(farScorePose.getHeading(), patrolStartPose.getHeading()).build();
        buildPatrolPaths();
    }

    public void buildPatrolPaths() {
        Pose start, end;
        start = patrolStartPose;
        end = new Pose(20.000, 52.000, Math.toRadians(180));
        patrolPaths.add(follower.pathBuilder().addPath(new BezierLine(start, end)).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build());
        patrolEndPoses.add(end);
        start = end;
        end = new Pose(20.000, 52.000, Math.toRadians(0));
        patrolPaths.add(follower.pathBuilder().addPath(new BezierLine(start, end)).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build());
        patrolEndPoses.add(end);
        start = end;
        end = new Pose(20.000, 11.000, Math.toRadians(0));
        patrolPaths.add(follower.pathBuilder().addPath(new BezierLine(start, end)).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build());
        patrolEndPoses.add(end);
        start = end;
        end = new Pose(40.000, 11.000, Math.toRadians(180));
        patrolPaths.add(follower.pathBuilder().addPath(new BezierLine(start, end)).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build());
        patrolEndPoses.add(end);
        start = end;
        end = new Pose(40.000, 52.000, Math.toRadians(180));
        patrolPaths.add(follower.pathBuilder().addPath(new BezierLine(start, end)).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build());
        patrolEndPoses.add(end);
        start = end;
        end = new Pose(40.000, 11.000, Math.toRadians(0));
        patrolPaths.add(follower.pathBuilder().addPath(new BezierLine(start, end)).setLinearHeadingInterpolation(start.getHeading(), end.getHeading()).build());
        patrolEndPoses.add(end);
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        SH = hardwareMap.get(DcMotorEx.class, "SH");
        MOZART = hardwareMap.get(DcMotorEx.class, "MOZART");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        RP = hardwareMap.get(Servo.class, "RP");
        LP = hardwareMap.get(Servo.class, "LP");
        washer = hardwareMap.get(CRServo.class, "washer");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        ClassifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "juju2");
        actionTimer = new Timer();
        patrolTimer = new Timer();
        SH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SH.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(250, 0.1, 30, 13));
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        MOZART.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RP.setPosition(0.1);
        LP.setPosition(1.0);
        limelight.pipelineSwitch(0);
        limelight.start();
        follower.setStartingPose(startPose);
        buildPaths();
        telemetry.addData("Status", "Far0 + Patrol Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        setState(0);
    }

    @Override
    public void loop() {
        follower.update();
        if (isIntakePermanentlyOn) {
            runIntakeLogic();
        }
        autonomousPathUpdate();
        telemetry.addData("Main State", mainState);
        telemetry.addData("Patrol Path", (currentPatrolPathIndex + 1));
        telemetry.addData("Patrol Time", "%.1f / 7.8", patrolTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    public void autonomousPathUpdate() {
        if (mainState >= 20 && mainState < 100 && patrolTimer.getElapsedTimeSeconds() > 7.8) {
            setState(100);
        }

        switch (mainState) {
            // --- Phase 1: Far0Blue Logic ---
            case 0: follower.followPath(path1_Preload, true); SH.setVelocity(calculateTicks(FAR_SHOT_RPM)); setState(1); break;
            case 1: if (!follower.isBusy())  { runContinuousShooterLogic(FAR_SHOT_RPM); if (actionTimer.getElapsedTimeSeconds() > 3.3) { stopShooting(); setState(2); } } else { actionTimer.resetTimer(); } break;
            case 2: isMozartBraked = false; follower.followPath(path5_Sip1, false); setState(3); break;
            case 3: runIntakeLogic(); if (!follower.isBusy()) setState(4); break;
            case 4: follower.followPath(path6_Sip2, false); setState(5); break;
            case 5: runIntakeLogic(); if (!follower.isBusy()) setState(6); break;
            case 6: runIntakeLogic();follower.followPath(path7_Sip3, true); setState(7); runIntakeLogic(); break;
            case 7: sleep(500); runIntakeLogic(); if (!follower.isBusy()) setState(8); break;
            case 8: follower.followPath(path8_Score, true); SH.setVelocity(calculateTicks(FAR_SHOT_RPM)); setState(9); break;
            case 9: if (!follower.isBusy()) { runContinuousShooterLogic(FAR_SHOT_RPM); if (actionTimer.getElapsedTimeSeconds() > 3.3) { stopShooting(); setState(10); } } else { actionTimer.resetTimer(); } break;

            // --- Phase 2: Transition ---
            case 10: follower.followPath(path10_Transition, true); setState(11); break;
            case 11:
                if (!follower.isBusy()) {
                    patrolTimer.resetTimer();
                    currentPatrolPathIndex = 0;
                    follower.followPath(patrolPaths.get(currentPatrolPathIndex), true);
                    setState(20);
                }
                break;

            // --- Phase 3: Intelligent Patrol ---
            case 20:
                if (!follower.isBusy()) {
                    currentPatrolPathIndex++;
                    if (currentPatrolPathIndex >= patrolPaths.size()) { setState(103); }
                    else { follower.followPath(patrolPaths.get(currentPatrolPathIndex), true); }
                    break;
                }

                Pose closestObject = findClosestObjectFieldPose();
                if (closestObject != null) {
                    isMozartBraked = false;
                    dynamicPath = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), closestObject)).setTangentHeadingInterpolation().build();
                    follower.followPath(dynamicPath, true);
                    runIntakeLogic();
                    setState(21);
                }
                break;

            case 21:
                runIntakeLogic();
                if (!follower.isBusy()) {
                    isIntakePermanentlyOn = true;
                    Pose endPoint = patrolEndPoses.get(currentPatrolPathIndex);
                    dynamicPath = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), endPoint)).setLinearHeadingInterpolation(follower.getPose().getHeading(), endPoint.getHeading()).build();
                    follower.followPath(dynamicPath, true);
                    setState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) { setState(20); }
                break;

            // --- Phase 4: Timeout Escape Sequence ---
            case 100:
                stopIntake();
                // --- 角度修正 ---
                dynamicPath = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), farScorePose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), farScorePose.getHeading())
                        .build();
                follower.followPath(dynamicPath, true);
                SH.setVelocity(calculateTicks(FAR_SHOT_RPM));
                setState(101);
                break;
            case 101: if (!follower.isBusy()) setState(102); break;
            case 102: if (!follower.isBusy()) { runContinuousShooterLogic(FAR_SHOT_RPM); if (actionTimer.getElapsedTimeSeconds() > 3.3) { stopShooting(); setState(103); } } else { actionTimer.resetTimer(); } break;
            case 103:
                // --- 角度修正 ---
                dynamicPath = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), finalParkPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), finalParkPose.getHeading())
                        .build();
                follower.followPath(dynamicPath, true);
                setState(104);
                break;
            case 104: if (!follower.isBusy()) setState(-1); break;
        }
    }

    public void setState(int state) {
        mainState = state;
        actionTimer.resetTimer();
    }

    // --- Helper Functions ---
    private void runContinuousShooterLogic(double targetRPM) {
        SH.setVelocity(calculateTicks(targetRPM));
        double currentRPM = getShooterRPM();
        if (Math.abs(currentRPM - targetRPM) <= 50) {
            MOZART.setPower(1.0); Hold.setPower(1.0); ClassifyServo.setPower(1.0); washer.setPower(1.0);
        } else {
            MOZART.setPower(0); Hold.setPower(0); ClassifyServo.setPower(0); washer.setPower(0);
        }
    }
    private void stopShooting() { SH.setPower(0); MOZART.setPower(0); Hold.setPower(0); ClassifyServo.setPower(0); washer.setPower(0); }
    private double getShooterRPM() { return (SH.getVelocity() * 60.0) / (TICKS_PER_REV * GEAR_RATIO); }
    private double calculateTicks(double rpm) { return (rpm * TICKS_PER_REV * GEAR_RATIO) / 60.0; }
    private void runIntakeLogic() {
        Intake.setPower(1.0); washer.setPower(1.0); Hold.setPower(1.0); ClassifyServo.setPower(1.0);
        if (!isMozartBraked && distanceSensor2.getDistance(DistanceUnit.MM) < 50) { isMozartBraked = true; }
        MOZART.setPower(isMozartBraked ? 0.0 : 1.0);
    }
    private void stopIntake() { Intake.setPower(0); washer.setPower(0); Hold.setPower(0); ClassifyServo.setPower(0); MOZART.setPower(0); }
    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
    private Pose findClosestObjectFieldPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getDetectorResults().isEmpty()) return null;
        Pose robotPose = follower.getPose();
        Pose closestPose = null;
        double minDistanceSq = Double.MAX_VALUE;
        for (LLResultTypes.DetectorResult dr : result.getDetectorResults()) {
            double angleToGoalRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + dr.getTargetYDegrees());
            if (Math.abs(Math.cos(angleToGoalRad)) < 0.01) continue;
            double relativeY_cm = (0 - CAMERA_HEIGHT_CM) / Math.tan(angleToGoalRad);
            double relativeX_cm = relativeY_cm * Math.tan(Math.toRadians(dr.getTargetXDegrees()));
            double relativeY_in = relativeY_cm * CM_TO_INCH;
            double relativeX_in = relativeX_cm * CM_TO_INCH;
            double distanceSq = relativeX_in * relativeX_in + relativeY_in * relativeY_in;
            if (distanceSq > MAX_TARGET_DISTANCE_IN * MAX_TARGET_DISTANCE_IN) continue;
            double objectFieldX = robotPose.getX() + (relativeY_in * Math.cos(robotPose.getHeading()) - relativeX_in * Math.sin(robotPose.getHeading()));
            double objectFieldY = robotPose.getY() + (relativeY_in * Math.sin(robotPose.getHeading()) + relativeX_in * Math.cos(robotPose.getHeading()));
            if (distanceSq < minDistanceSq) { minDistanceSq = distanceSq; closestPose = new Pose(objectFieldX, objectFieldY); }
        }
        return closestPose;
    }
}