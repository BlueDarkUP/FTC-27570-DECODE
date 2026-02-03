package org.firstinspires.ftc.teamcode.auto.classify;

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
import org.firstinspires.ftc.teamcode.drive.LedModuleDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "蓝方18", group = "Main")
public class BlueCLS extends OpMode {

    private Follower follower;
    private Timer pathTimer, cycleTimer, actionTimer;
    private int pathState = 0;

    private final double FirstShootingPower = 2600.0;
    private final double NormalShootingPower = 2730.0;
    private final double IdleShootingPower = 1500.0;

    private DcMotorEx SH, HS, Intake, Mozart;
    private CRServo Hold;
    private Servo servoRP = null;
    private Servo servoLP = null;
    private DigitalChannel juju;

    private Servo lightLeft, lightRight;

    private enum LightState { IDLE, TRANSITIONING, ANIMATING }
    private LightState currentLightState = LightState.IDLE;
    private ElapsedTime lightTimer = new ElapsedTime();

    LedModuleDriver led;
    final char ID = 'A';

    private final int HEX_CYAN    = 0x00FFFF;
    private final int HEX_MAGENTA = 0xFF00FF;
    private final int HEX_GOLD    = 0xFFD700;
    private final int HEX_ORANGE  = 0xFF4500;
    private final int HEX_GREEN   = 0x00FF00;
    private final int HEX_WHITE   = 0xFFFFFF;
    private final int HEX_RED     = 0xFF0000;

    private String lastLedState = "";

    private double animationMin = 0.0, animationMax = 0.0, animationDurationSec = 1.0;
    private boolean isFadingUp = true;

    private double transitionFromPos = 0.0, transitionToPos = 0.0, targetMin = 0.0, targetMax = 0.0;
    private static final double TRANSITION_DURATION_SEC = 0.5;

    private static final double C_OFF = 0.1;
    private static final double C_RED = 0.279;
    private static final double C_ORANGE = 0.333;
    private static final double C_YELLOW = 0.388;
    private static final double C_SAGE = 0.444;
    private static final double C_GREEN = 0.500;
    private static final double C_Azure = 0.555;
    private static final double C_BLUE = 0.611;
    private static final double C_Indigo = 0.666;
    private static final double C_VIOLET = 0.722;
    private static final double C_WHITE = 0.8;

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

    private final Pose startPose = new Pose(27.495, 132.350, Math.toRadians(-36));
    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13;

    @Override
    public void init() {
        pathTimer = new Timer();
        cycleTimer = new Timer();
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

        led = hardwareMap.get(LedModuleDriver.class, "leddd");
        led.assignId(ID);
        led.setWaveMode('A', 0xFF00FF, 0x00FFFF, 200, 60);

        juju.setMode(DigitalChannel.Mode.INPUT);
        Mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        servoLP.setPosition(0.0);
        servoRP.setPosition(0.925);

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(0);
        setLightAnimation(C_BLUE, C_Indigo, 1.5);
    }

    @Override
    public void loop() {
        updateShooterPID();

        if (isShootingTaskActive) {
            runShootingSequence();
        } else {
            updateIntakeLogic();
        }

        follower.update();
        autonomousPathUpdate();
        updateLights();
        updateSmartLed();

        telemetry.addData("State", pathState);
        telemetry.addData("Actual RPM", getShooterRPM());
        telemetry.addData("Loop Time", cycleTimer.getElapsedTimeSeconds());
        telemetry.addData("LED State", lastLedState);
        telemetry.update();
    }

    private void updateSmartLed() {
        String reqState = "";
        if (isShootingTaskActive) {
            reqState = "SHOOT";
            if (!reqState.equals(lastLedState)) lastLedState = reqState;
            return;
        }
        if (targetShooterRPM > 1000) {
            reqState = "FLYWHEEL";
            if (!reqState.equals(lastLedState)) lastLedState = reqState;
        }
        if (isIntakeActive) {
            if (hasCaughtObject) {
                reqState = "CAUGHT";
                if (!reqState.equals(lastLedState)) lastLedState = reqState;
            } else {
                reqState = "INTAKING";
                if (!reqState.equals(lastLedState)) lastLedState = reqState;
            }
            return;
        }
        if (reqState.isEmpty()) {
            reqState = "CRUISE";
            if (!reqState.equals(lastLedState)) lastLedState = reqState;
        }
    }

    public void setLightAnimation(double min, double max, double durationSec) {
        if (targetMin == min && targetMax == max && Math.abs(animationDurationSec - durationSec) < 0.01) return;
        this.targetMin = min;
        this.targetMax = max;
        this.animationDurationSec = durationSec;
        this.transitionFromPos = lightLeft.getPosition();
        this.transitionToPos = this.targetMin;
        this.currentLightState = LightState.TRANSITIONING;
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
                    animationMin = targetMin;
                    animationMax = targetMax;
                    isFadingUp = true;
                    currentLightState = LightState.ANIMATING;
                    lightTimer.reset();
                }
                break;
            case ANIMATING:
                double animationProgress = Math.min(elapsedTime / animationDurationSec, 1.0);
                if (isFadingUp) currentPosition = animationMin + (animationMax - animationMin) * animationProgress;
                else currentPosition = animationMax - (animationMax - animationMin) * animationProgress;
                if (animationProgress >= 1.0) {
                    isFadingUp = !isFadingUp;
                    lightTimer.reset();
                }
                break;
        }
        lightLeft.setPosition(currentPosition);
        lightRight.setPosition(currentPosition);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetShooterRPM = FirstShootingPower;
                follower.followPath(Path1, false);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setLightAnimation(C_RED, C_ORANGE, 0.5);
                    shooting(500, false);
                    servoLP.setPosition(0.30);
                    servoRP.setPosition(0.625);
                    hasCaughtObject = false;
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    setPathState(1145);
                }
                break;

            case 1145:
                if (!isShootingTaskActive) {
                    targetShooterRPM = IdleShootingPower;
                    setLightAnimation(C_SAGE, C_GREEN, 1);
                    follower.followPath(Path2, false);
                    setPathState(300);
                }
                break;
            case 300:
                if (!follower.isBusy()) {
                    setLightAnimation(C_Azure, C_VIOLET, 2);
                    setIntake(true);
                    servoLP.setPosition(0.30);
                    servoRP.setPosition(0.625);
                    follower.setMaxPower(0.9);
                    follower.followPath(Path3, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.setMaxPower(1);
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                // 非阻塞等待 300ms 稳定后再射击
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(49);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 49:
                if (!isShootingTaskActive) {
                    setLightAnimation(C_ORANGE, C_SAGE, 0.5);
                    cycleTimer.resetTimer();
                    setPathState(50);
                }
                break;
            case 50:
                if (cycleTimer.getElapsedTimeSeconds() >= 10.0) {
                    setIntake(false);
                    targetShooterRPM = IdleShootingPower;
                    setPathState(7);
                } else {
                    targetShooterRPM = IdleShootingPower;
                    hasCaughtObject = false;
                    setIntake(true);
                    follower.followPath(Path5, true);
                    setPathState(51);
                }
                break;
            case 51:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        targetShooterRPM = NormalShootingPower;
                        follower.followPath(Path6, true);
                        setPathState(55);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 55:
                // 非阻塞等待 300ms 稳定后再射击
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(53);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 53:
                if (!isShootingTaskActive) setPathState(50);
                break;
            case 7:
                setLightAnimation(C_Azure, C_VIOLET, 2);
                follower.followPath(Path7, false);
                setPathState(8);
                break;
            case 8:
                if (!follower.isBusy()) {
                    setIntake(true);
                    follower.setMaxPower(0.9);
                    follower.followPath(Path8, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.setMaxPower(1);
                    follower.followPath(Path9, true);
                    setPathState(10);
                }
                break;
            case 10:
                // 非阻塞等待 300ms 稳定后再射击
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(101);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 101:
                if (!isShootingTaskActive) {
                    targetShooterRPM = IdleShootingPower;
                    setLightAnimation(C_Azure, C_VIOLET, 2);
                    follower.followPath(Path10, false);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setIntake(true);
                    follower.setMaxPower(0.9);
                    follower.followPath(Path11, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    setLightAnimation(C_BLUE, C_Indigo, 1.5);
                    follower.setMaxPower(1);
                    follower.followPath(Path12, true);
                    setPathState(13);
                }
                break;
            case 13:
                // 非阻塞等待 300ms 稳定后再射击
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                        setLightAnimation(C_RED, C_ORANGE, 0.5);
                        shooting(500, false);
                        hasCaughtObject = false;
                        setPathState(131);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;
            case 131:
                if (!isShootingTaskActive) {
                    setIntake(false);
                    targetShooterRPM = 0;
                    setLightAnimation(C_RED, C_VIOLET, 0.8);
                    follower.followPath(Path13, true);
                    setPathState(14);
                }
                break;
            case 14:
                led.turnOff(ID);
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    public void shooting(long millis, boolean holdChassis) {
        shootTimeLimitSec = millis / 1000.0;
        isShootingTaskActive = true;
        isChassisPausedByShot = holdChassis;
        shootActionTimer.reset();
        if (holdChassis) follower.pausePathFollowing();
    }

    private void runShootingSequence() {
        if (shootActionTimer.seconds() < shootTimeLimitSec) {
            Intake.setPower(1.0); Mozart.setPower(0.85); Hold.setPower(1.0);
        } else {
            isShootingTaskActive = false;
            Intake.setPower(0); Mozart.setPower(0); Hold.setPower(0);
            if (isChassisPausedByShot) follower.resumePathFollowing();
        }
    }

    public void setIntake(boolean active) {
        if (active && !isIntakeActive) hasCaughtObject = false;
        isIntakeActive = active;
        if (active) {
            hasCaughtObject = false;
        }
    }

    private void updateIntakeLogic() {
        if (isIntakeActive) {
            Intake.setPower(1.0); Hold.setPower(1.0);
            if (!hasCaughtObject && juju.getState()) hasCaughtObject = true;
            Mozart.setPower(hasCaughtObject ? 0 : 0.6);
        } else {
            Intake.setPower(0); Hold.setPower(0); Mozart.setPower(0);
        }
    }

    private void updateShooterPID() {
        double currentVelTPS = SH.getVelocity();
        double targetVelTPS = (targetShooterRPM * TICKS_PER_REV) / 60.0;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt == 0) dt = 1e-9;
        double errorTPS = targetVelTPS - currentVelTPS;
        double derivative = (errorTPS - shooterLastError) / dt;
        shooterLastError = errorTPS;
        double finalPower = (SHOOTER_F * targetVelTPS) + (SHOOTER_P * errorTPS) + (SHOOTER_D * derivative);
        if (targetShooterRPM == 0) finalPower = 0;
        SH.setPower(finalPower); HS.setPower(finalPower);
    }

    private double getShooterRPM() { return (SH.getVelocity() * 60.0) / TICKS_PER_REV; }

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(27.495, 132.350), new Pose(55.951, 93.204))).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-50)).build();
        Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(55.951, 93.204), new Pose(44.505, 66))).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(180)).build();
        Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(44.505, 66), new Pose(14.500, 66))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
        Path4 = follower.pathBuilder().addPath(new BezierCurve(new Pose(14.500, 66), new Pose(55, 77))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-55)).build();
        Path5 = follower.pathBuilder().addPath(new BezierCurve(new Pose(55, 77), new Pose(40.000, 25.000),new Pose(8, 58))).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(160)).build();
        Path6 = follower.pathBuilder().addPath(new BezierCurve(new Pose(8, 58), new Pose(55, 77))).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(-55)).build();
        Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(55, 77), new Pose(46.602, 86.5))).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(180)).build();
        Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(46.602, 86.5), new Pose(22.5, 86.5))).setTangentHeadingInterpolation().build();
        Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(24.000, 86.5), new Pose(55, 77))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-55)).build();
        Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(55, 77), new Pose(43.107, 44))).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(180)).build();
        Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(43.107, 44), new Pose(17, 44))).setTangentHeadingInterpolation().build();
        Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(17, 44), new Pose(55, 77))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-55)).build();
        Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(55, 77), new Pose(57.553, 56.388))).setLinearHeadingInterpolation(Math.toRadians(-55), Math.toRadians(180)).build();
    }
}