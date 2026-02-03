package org.firstinspires.ftc.teamcode.auto.Red;

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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "红方远点", group = "Main")
public class RedFarAutoCycle extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState = 0;
    private int loopCount = 0;

    private final double FarShootingRPM = 3250.0;
    private final double IdleRPM = 1500.0;
    private final double RPM_TOLERANCE = 300.0;

    private double currentSlewRPM = 0;
    private final double RPM_DOWN_STEP = 8.0;

    private DcMotorEx SH, HS, Intake, Mozart;
    private CRServo Hold;
    private Servo servoRP = null, servoLP = null;
    private DigitalChannel juju;
    private Servo lightLeft, lightRight;

    private enum LightState { IDLE, TRANSITIONING, ANIMATING }
    private RedFarAutoCycle.LightState currentLightState = RedFarAutoCycle.LightState.IDLE;
    private ElapsedTime lightTimer = new ElapsedTime();

    private double animationMin = 0.0, animationMax = 0.0, animationDurationSec = 1.0;
    private boolean isFadingUp = true;

    private double transitionFromPos = 0.0, transitionToPos = 0.0, targetMin = 0.0, targetMax = 0.0;
    private static final double TRANSITION_DURATION_SEC = 0.5;

    private static final double C_RED = 0.279;
    private static final double C_ORANGE = 0.333;
    private static final double C_SAGE = 0.444;
    private static final double C_GREEN = 0.500;
    private static final double C_Azure = 0.555;
    private static final double C_BLUE = 0.611;
    private static final double C_Indigo = 0.666;
    private static final double C_VIOLET = 0.722;

    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double targetShooterRPM = 0;
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.007, SHOOTER_F = 0.0003, SHOOTER_D = 0.000001, TICKS_PER_REV = 28.0;

    private boolean isIntakeActive = false;
    private boolean hasCaughtObject = false;
    private ElapsedTime shootActionTimer = new ElapsedTime();
    private double shootTimeLimitSec = 0;
    private boolean isShootingTaskActive = false;
    private boolean isChassisPausedByShot = false;

    private final Pose startPose = new Pose(88.000, 8.000, Math.toRadians(270));
    private PathChain Path1, Path2, PathWiggle, Path3, Path4;

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        shooterPidTimer.reset();

        SH = hardwareMap.get(DcMotorEx.class, "SH");
        HS = hardwareMap.get(DcMotorEx.class, "HS");
        SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        HS.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Mozart = hardwareMap.get(DcMotorEx.class, "MOZART");
        Hold = hardwareMap.get(CRServo.class, "Hold");
        juju = hardwareMap.get(DigitalChannel.class, "bigjuju");
        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");

        lightLeft = hardwareMap.get(Servo.class, "lightLeft");
        lightRight = hardwareMap.get(Servo.class, "lightRight");

        juju.setMode(DigitalChannel.Mode.INPUT);
        Mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        servoLP.setPosition(0.925);
        servoRP.setPosition(0.03);

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
        setLightAnimation(C_RED, C_ORANGE, 1.5);
    }

    @Override
    public void loop() {
        updateShooterPID();
        if (isShootingTaskActive) runShootingSequence(); else updateIntakeLogic();

        follower.update();
        autonomousPathUpdate();
        updateLights();
        telemetry.addData("State", pathState);
        telemetry.addData("Loop", loopCount);
        telemetry.addData("T-Value", follower.getCurrentTValue());
        telemetry.addData("Current RPM", "%.0f", getShooterRPM());
        telemetry.update();
    }

    public void setLightAnimation(double min, double max, double durationSec) {
        if (targetMin == min && targetMax == max && Math.abs(animationDurationSec - durationSec) < 0.01) return;
        this.targetMin = min;
        this.targetMax = max;
        this.animationDurationSec = durationSec;
        this.transitionFromPos = lightLeft.getPosition();
        this.transitionToPos = this.targetMin;
        this.currentLightState = RedFarAutoCycle.LightState.TRANSITIONING;
        this.lightTimer.reset();
    }

    public void updateLights() {
        double currentPosition = lightLeft.getPosition();
        double elapsedTime = lightTimer.seconds();
        switch (currentLightState) {
            case IDLE: break;
            case TRANSITIONING:
                double transitionProgress = Math.min(elapsedTime / TRANSITION_DURATION_SEC, 1.0);
                currentPosition = transitionFromPos + (transitionToPos - transitionFromPos) * transitionProgress;
                if (transitionProgress >= 1.0) {
                    currentPosition = transitionToPos;
                    animationMin = targetMin; animationMax = targetMax;
                    isFadingUp = true;
                    currentLightState = RedFarAutoCycle.LightState.ANIMATING;
                    lightTimer.reset();
                }
                break;
            case ANIMATING:
                double animationProgress = Math.min(elapsedTime / animationDurationSec, 1.0);
                if (isFadingUp) currentPosition = animationMin + (animationMax - animationMin) * animationProgress;
                else currentPosition = animationMax - (animationMax - animationMin) * animationProgress;
                if (animationProgress >= 1.0) { isFadingUp = !isFadingUp; lightTimer.reset(); }
                break;
        }
        lightLeft.setPosition(currentPosition);
        lightRight.setPosition(currentPosition);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setLightAnimation(C_RED, C_ORANGE, 1.5);
                targetShooterRPM = FarShootingRPM;
                follower.followPath(Path1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(1000, false);
                        setPathState(2);
                    }
                } else { actionTimer.resetTimer(); }
                break;
            case 2:
                if (!isShootingTaskActive) {
                    setLightAnimation(C_ORANGE, C_SAGE, 0.5);
                    targetShooterRPM = IdleRPM;
                    loopCount = 0;
                    setPathState(10);
                }
                break;

            case 10:
                if (loopCount < 3) {
                    setLightAnimation(C_Azure, C_VIOLET, 2);
                    setIntake(true);
                    follower.setMaxPower(0.85);
                    follower.followPath(Path2, false);
                    setPathState(11);
                } else {
                    setPathState(20);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(PathWiggle, false);
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.setMaxPower(1.0);
                    follower.followPath(Path3, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    targetShooterRPM = FarShootingRPM;
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(1000, false);
                        setPathState(13);
                    }
                } else {
                    if(follower.getCurrentTValue() > 0.5) targetShooterRPM = FarShootingRPM;
                    actionTimer.resetTimer();
                }
                break;

            case 13:
                if (!isShootingTaskActive) {
                    setLightAnimation(C_ORANGE, C_SAGE, 0.5);
                    targetShooterRPM = IdleRPM;
                    setIntake(false);
                    loopCount++;
                    setPathState(10);
                }
                break;

            case 20:
                targetShooterRPM = 0;
                setIntake(false);
                setLightAnimation(C_VIOLET, C_Indigo, 0.8);
                follower.followPath(Path4, true);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    private void runShootingSequence() {
        if (shootActionTimer.seconds() < shootTimeLimitSec) {
            Intake.setPower(1.0);
            Hold.setPower(1.0);
            if (Math.abs(getShooterRPM() - targetShooterRPM) <= RPM_TOLERANCE) {
                Mozart.setPower(0.6);
            } else { Mozart.setPower(0); }
        } else {
            isShootingTaskActive = false;
            Intake.setPower(0); Mozart.setPower(0); Hold.setPower(0);
            if (isChassisPausedByShot) follower.resumePathFollowing();
        }
    }

    public void shooting(long millis, boolean holdChassis) {
        shootTimeLimitSec = millis / 1000.0;
        isShootingTaskActive = true;
        isChassisPausedByShot = holdChassis;
        shootActionTimer.reset();
        if (holdChassis) follower.pausePathFollowing();
    }

    public void setIntake(boolean active) {
        if (active && !isIntakeActive) hasCaughtObject = false;
        isIntakeActive = active;
    }

    private void updateIntakeLogic() {
        if (isIntakeActive) {
            Intake.setPower(1.0); Hold.setPower(1.0);
            if (!hasCaughtObject && juju.getState()) hasCaughtObject = true;
            Mozart.setPower(hasCaughtObject ? 0 : 0.6);
        } else { Intake.setPower(0); Hold.setPower(0); Mozart.setPower(0); }
    }

    private void updateShooterPID() {
        if (currentSlewRPM > targetShooterRPM) {
            currentSlewRPM -= RPM_DOWN_STEP;
            if (currentSlewRPM < targetShooterRPM) currentSlewRPM = targetShooterRPM;
        } else { currentSlewRPM = targetShooterRPM; }
        double targetVelTPS = (currentSlewRPM * TICKS_PER_REV) / 60.0;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt == 0) dt = 1e-9;
        double errorTPS = targetVelTPS - SH.getVelocity();
        double derivative = (errorTPS - shooterLastError) / dt;
        shooterLastError = errorTPS;
        double finalPower = (SHOOTER_F * targetVelTPS) + (SHOOTER_P * errorTPS) + (SHOOTER_D * derivative);
        if (targetShooterRPM == 0 && currentSlewRPM == 0) finalPower = 0;
        SH.setPower(finalPower); HS.setPower(finalPower);
    }

    private double getShooterRPM() { return (SH.getVelocity() * 60.0) / TICKS_PER_REV; }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88.000, 8.000), // 144 - 56
                        new Pose(88.000, 14.000) // 144 - 56
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(248)) // 180 - (-68)
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(88.000, 14.000),
                        new Pose(94.000, 70.000), // 144 - 50
                        new Pose(135.000, 10.000) // 144 - 9
                ))
                .setLinearHeadingInterpolation(Math.toRadians(248), Math.toRadians(0)) // 180 - 180
                .build();

        PathWiggle = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(135.000, 10.000), new Pose(129.000, 10.000))) // 144 - 15
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Pose(129.000, 10.000), new Pose(135.000, 10.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Pose(135.000, 10.000), new Pose(129.000, 10.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Pose(129.000, 10.000), new Pose(135.000, 10.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(135.000, 10.000),
                        new Pose(94.000, 70.000),
                        new Pose(88.000, 14.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(248))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88.000, 14.000),
                        new Pose(104.000, 14.000) // 144 - 40
                ))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }
}