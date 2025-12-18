package org.firstinspires.ftc.teamcode.auto.FarAndDeadEye;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Intelligent Patrol (Distance Sensor)", group = "PedroPathing Advanced")
public class IntelligentPatrol extends OpMode {

    private enum PatrolState { PATROLLING, FETCHING, RESUMING, HOLDING }
    private PatrolState currentState = PatrolState.PATROLLING;
    private Follower follower;
    private Limelight3A limelight;
    private DcMotorEx MOZART, Intake;
    private CRServo washer, Hold, ClassifyServo;
    private DistanceSensor distanceSensor2;

    private List<PathChain> patrolPaths = new ArrayList<>();
    private List<Pose> patrolEndPoses = new ArrayList<>();
    private int currentPatrolPathIndex = 0;
    private boolean isIntakePermanentlyOn = false;
    private boolean isObjectClose = false;
    private static final double CAMERA_HEIGHT_CM = 31.5;
    private static final double CAMERA_MOUNT_ANGLE_DEG = -53.5;
    private static final double CM_TO_INCH = 1.0 / 2.54;
    private static final double MAX_TARGET_DISTANCE_IN = 100.0;

    private final Pose startPose = new Pose(20.000, 11.000, Math.toRadians(180));

    public void buildPatrolPaths() {
        Pose start, end;
        start = new Pose(20.000, 11.000, Math.toRadians(180));
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
        MOZART = hardwareMap.get(DcMotorEx.class, "MOZART");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        washer = hardwareMap.get(CRServo.class, "washer");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        ClassifyServo = hardwareMap.get(CRServo.class, "ClassifyServo");

        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "juju2");

        limelight.pipelineSwitch(0);
        limelight.start();
        follower.setStartingPose(startPose);
        buildPatrolPaths();
        telemetry.addData("Status", "Intelligent Patrol Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(patrolPaths.get(currentPatrolPathIndex), true);
        currentState = PatrolState.PATROLLING;
    }

    @Override
    public void loop() {
        follower.update();
        if (isIntakePermanentlyOn) {
            runIntakeLogic();
        }

        switch (currentState) {
            case PATROLLING:
                if (!follower.isBusy()) {
                    currentPatrolPathIndex++;
                    if (currentPatrolPathIndex >= patrolPaths.size()) {
                        currentState = PatrolState.HOLDING;
                        stopIntake();
                    } else {
                        follower.followPath(patrolPaths.get(currentPatrolPathIndex), true);
                    }
                    break;
                }

                Pose closestObjectPose = findClosestObjectFieldPose();
                if (closestObjectPose != null) {
                    isObjectClose = false;

                    PathChain fetchPath = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), closestObjectPose))
                            .setTangentHeadingInterpolation()
                            .build();
                    follower.followPath(fetchPath, true);
                    runIntakeLogic();
                    currentState = PatrolState.FETCHING;
                }
                break;

            case FETCHING:
                runIntakeLogic();
                if (!follower.isBusy()) {
                    isIntakePermanentlyOn = true;
                    Pose patrolEndPoint = patrolEndPoses.get(currentPatrolPathIndex);
                    PathChain resumePath = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), patrolEndPoint))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), patrolEndPoint.getHeading())
                            .build();
                    follower.followPath(resumePath, true);
                    currentState = PatrolState.RESUMING;
                }
                break;

            case RESUMING:
                if (!follower.isBusy()) {
                    currentState = PatrolState.PATROLLING;
                }
                break;

            case HOLDING:
                break;
        }

        telemetry.addData("Current State", currentState.name());
        telemetry.addData("Patrol Path", (currentPatrolPathIndex + 1) + " / " + patrolPaths.size());
        telemetry.addData("Object Close", isObjectClose);
        telemetry.addData("Distance (mm)", distanceSensor2.getDistance(DistanceUnit.MM));
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private Pose findClosestObjectFieldPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getDetectorResults().isEmpty()) {
            return null;
        }
        Pose robotPose = follower.getPose();
        double robotHeadingRad = robotPose.getHeading();
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
            if (distanceSq > MAX_TARGET_DISTANCE_IN * MAX_TARGET_DISTANCE_IN) {
                continue;
            }
            double objectFieldX = robotPose.getX() + (relativeY_in * Math.cos(robotHeadingRad) - relativeX_in * Math.sin(robotHeadingRad));
            double objectFieldY = robotPose.getY() + (relativeY_in * Math.sin(robotHeadingRad) + relativeX_in * Math.cos(robotHeadingRad));
            if (distanceSq < minDistanceSq) {
                minDistanceSq = distanceSq;
                closestPose = new Pose(objectFieldX, objectFieldY);
            }
        }
        return closestPose;
    }

    private void runIntakeLogic() {
        Intake.setPower(1.0);
        washer.setPower(1.0);
        Hold.setPower(1.0);
        ClassifyServo.setPower(1.0);

        if (!isObjectClose) {
            double dist_mm = distanceSensor2.getDistance(DistanceUnit.MM);
            if (dist_mm < 50) {
                isObjectClose = true;
            }
        }

        if (isObjectClose) {
            MOZART.setPower(0.0);
        } else {
            MOZART.setPower(1.0);
        }
    }

    private void stopIntake() {
        Intake.setPower(0);
        MOZART.setPower(0);
        washer.setPower(0);
        Hold.setPower(0);
        ClassifyServo.setPower(0);
    }
}