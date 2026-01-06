package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name="Qual1AutoFar", group="Robot")

public class Qual1AutoFar extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 1;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        Pose2d initialPose = new Pose2d(0, -60, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Define how the hub is mounted on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);


        sleep(2000);

        while (!isStopRequested() && !opModeIsActive()) {
            // int position = visionOutputPosition;
            telemetry.addLine("initialized!");
            telemetry.update();
        }

        telemetry.addLine("In Starting Position");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        Action trajectoryAction = drive.actionBuilder(initialPose)
                .lineToX(-40)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction
                )
        );

    }
}


