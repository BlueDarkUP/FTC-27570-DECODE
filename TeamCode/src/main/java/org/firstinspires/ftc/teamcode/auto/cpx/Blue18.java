package org.firstinspires.ftc.teamcode.auto.cpx;

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

@Autonomous(name = "Final: 13段路径全动作自动程序", group = "Main")
public class Blue18 extends OpMode {

    private Follower follower;
    private Timer pathTimer, cycleTimer, actionTimer;
    private int pathState = 0;

    private final double FirstShootingPower = 2800.0;
    private final double NormalShootingPower = 2400.0;

    private DcMotorEx SH, HS, Intake, Mozart;
    private CRServo Hold;
    private Servo servoRP = null;
    private Servo servoLP = null;
    private DigitalChannel juju;

    private ElapsedTime shooterPidTimer = new ElapsedTime();
    private double targetShooterRPM = 0;
    private double shooterLastError = 0;
    private static final double SHOOTER_P = 0.006, SHOOTER_F = 0.0004, SHOOTER_D = 0.00001, TICKS_PER_REV = 28.0;

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
        juju = hardwareMap.get(DigitalChannel.class, "juju");
        servoRP = hardwareMap.get(Servo.class, "RP");
        servoLP = hardwareMap.get(Servo.class, "LP");
        juju.setMode(DigitalChannel.Mode.INPUT);
        Mozart.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        servoLP.setPosition(0.60);
        servoRP.setPosition(0.325);
        buildPaths();
    }

    @Override
    public void start() { setPathState(0); }

    @Override
    public void loop() {
        updateShooterPID();
        if (isShootingTaskActive) runShootingSequence(); else updateIntakeLogic();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Actual RPM", getShooterRPM());
        telemetry.addData("Loop Time", cycleTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetShooterRPM = FirstShootingPower;
                follower.followPath(Path1, false);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(1145);
                }
                break;

            case 1145:
                if (!isShootingTaskActive) {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    setIntake(true);
                    servoLP.setPosition(0.30);
                    servoRP.setPosition(0.625);
                    follower.followPath(Path3, false);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(49);
                }
                break;

            case 49:
                if (!isShootingTaskActive) {
                    cycleTimer.resetTimer();
                    setPathState(50);
                }
                break;

            case 50:
                if (cycleTimer.getElapsedTimeSeconds() >= 10.0) {
                    setIntake(false);
                    setPathState(7);
                } else {
                    setIntake(true);
                    follower.followPath(Path5, false);
                    setPathState(51);
                }
                break;

            case 51:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 2.0) {
                        follower.followPath(Path6, true);
                        setPathState(52);
                    }
                } else {
                    actionTimer.resetTimer();
                }
                break;

            case 52:
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(53);
                }
                break;

            case 53:
                if (!isShootingTaskActive) setPathState(50);
                break;

            case 7:
                follower.followPath(Path7, true);
                setPathState(8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    setIntake(true);
                    follower.followPath(Path8, false);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.followPath(Path9, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(101);
                }
                break;

            case 101:
                if (!isShootingTaskActive) {
                    follower.followPath(Path10, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    setIntake(true);
                    follower.followPath(Path11, false);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    targetShooterRPM = NormalShootingPower;
                    follower.followPath(Path12, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    shooting(500, false);
                    setPathState(131);
                }
                break;

            case 131:
                if (!isShootingTaskActive) {
                    setIntake(false);
                    targetShooterRPM = 0;
                    follower.followPath(Path13, true);
                    setPathState(14);
                }
                break;

            case 14:
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
            Intake.setPower(1.0); Mozart.setPower(0.8); Hold.setPower(1.0);
        } else {
            isShootingTaskActive = false;
            Intake.setPower(0); Mozart.setPower(0); Hold.setPower(0);
            if (isChassisPausedByShot) follower.resumePathFollowing();
        }
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
        Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(27.495, 132.350), new Pose(58.951, 93.204))).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-45)).build();
        Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(58.951, 93.204), new Pose(44.505, 63.5))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
        Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(44.505, 63.5), new Pose(12.500, 63.5))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
        Path4 = follower.pathBuilder().addPath(new BezierCurve(new Pose(12.500, 63.5), new Pose(41.942, 45), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48)).build();
        Path5 = follower.pathBuilder().addPath(new BezierCurve(new Pose(60, 84), new Pose(43, 15), new Pose(7, 62.5))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(150)).build();
        Path6 = follower.pathBuilder().addPath(new BezierCurve(new Pose(7.5, 62.5), new Pose(41.942, 45), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(-48)).build();
        Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 84), new Pose(46.602, 85))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
        Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(46.602, 85), new Pose(22.000, 85))).setTangentHeadingInterpolation().build();
        Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(22.000, 85), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48)).build();
        Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 84), new Pose(43.107, 38))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
        Path11 = follower.pathBuilder().addPath(new BezierLine(new Pose(43.107, 38), new Pose(12.500, 38))).setTangentHeadingInterpolation().build();
        Path12 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.500, 38), new Pose(60, 84))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-48)).build();
        Path13 = follower.pathBuilder().addPath(new BezierLine(new Pose(60, 84), new Pose(57.553, 56.388))).setLinearHeadingInterpolation(Math.toRadians(-48), Math.toRadians(180)).build();
    }
}