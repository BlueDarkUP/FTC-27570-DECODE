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
            .forwardZeroPowerAcceleration(-37.9)
            .lateralZeroPowerAcceleration(-59.02)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.012, 0, 0.001, 0.1))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.0, 0.02))

            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.0, 0.1))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.0, 0.01))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.0,0.7,0.08))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.08,0.0,0.00,0.65,0.0));


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.0283)
            .strafePodX(0.44488)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
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
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftRearMotorName("LB")
            .leftFrontMotorName("LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81)
            .yVelocity(65);

    }
