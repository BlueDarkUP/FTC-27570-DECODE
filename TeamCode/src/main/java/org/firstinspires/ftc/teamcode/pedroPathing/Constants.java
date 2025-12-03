package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.3498)
            .forwardZeroPowerAcceleration(-31.7011407118028235)
            .lateralZeroPowerAcceleration(-56.27560468087312)

            .centripetalScaling(0.005)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.018,0.0,0.00025,0.65,0.07))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0.0018,0.7,0.01))

            .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.2, 0.11))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.08,0.022))

            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.01, 0.12))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05,0.00001,0.02,0.023));


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(160)
            .strafePodX(-75)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontDrive")
            .rightRearMotorName("RightBehindDrive")
            .leftRearMotorName("LeftBehindDrive")
            .leftFrontMotorName("LeftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(81)
            .yVelocity(81);

    }
