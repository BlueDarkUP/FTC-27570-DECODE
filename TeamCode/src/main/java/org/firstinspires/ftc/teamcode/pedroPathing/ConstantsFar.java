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

public class ConstantsFar {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-40.7)
            .lateralZeroPowerAcceleration(-68)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.027, 0, 0.002, 0.15))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.03,0,0.0055,0))

            .headingPIDFCoefficients(new PIDFCoefficients(0.35, 0.0001, 0.005, 0.1))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5,0,0.001,0.01))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.0005,0.6,0.08))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.2,0,0.0001,0.6,0.0))

            .centripetalScaling(0.5);



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
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(86)
            .yVelocity(64);

    }
