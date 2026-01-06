package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

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

@Config
@Autonomous(name="Qual1AutoNear", group="Robot")
public class Qual1AutoNear extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private Servo scoop, ramp;
    private CRServo intake, transition;


    //common power declarations
    double intakePow = 0.5;
    double transPow = 0.5;
    private double flywheelPow = 0.0;

    //common position declarations
    double scoopDownPos = 0.6;
    double scoopUpPos = 0.0;
    double rampClosePos = 0.0;
    double rampMidPos = 0.3;
    double rampFarPos = 0.5;

    double TPR = 104;
    double rpm;
    double TPS;

    // non chassis declarations: flywheel, intake, transition, scoop
    public class Flywheel {
        private DcMotorEx flywheel;

        public Flywheel (HardwareMap hardwareMap) {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public void setVelocity(double power) {
            flywheel.setVelocity(power);
        }

        public double getVelocity() {
            return flywheel.getVelocity();
        }

        public void stop() {
            flywheel.setPower(0.0);
        }

    }

    public class intake {
        private Servo intake;

        public intake (HardwareMap hardwareMap) {
            intake = hardwareMap.get(Servo.class, "intake");
        }
    }

    public class transition {
        private Servo transition;

        public transition (HardwareMap hardwareMap) {
            transition = hardwareMap.get(Servo.class, "transition");
        }
    }

    public class scoop {
        private Servo scoop;

        public scoop (HardwareMap hardwareMap) {
            scoop = hardwareMap.get(Servo.class, "scoop");
        }
    }

    // automated flywheel cycle
    public class FlywheelCycle implements Action {
        private Flywheel flywheel;
        private Servo scoop;
        private long startTime;

        // checks if the flywheel motor has been powered on
        private boolean initialized = false;

        public FlywheelCycle(Flywheel flywheel, Servo scoop) {
            this.flywheel = flywheel;
            this.scoop = scoop;
        }

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                flywheel.setVelocity(0);
                initialized = true;
                startTime = System.currentTimeMillis(); // <-- define start time here
            }

            long elapsed = System.currentTimeMillis() - startTime;

            // checks flywheel's current position
            double rpm = Math.abs((flywheel.getVelocity()/104) * 60.0);
            flywheel.setVelocity(1100);
            packet.put("flywheelVelocity", rpm);
            if (rpm < 1000.0) {
                // true causes the action to rerun, so with a velocity of < 1000 it keeps going
                return true;
            } else {
                telemetry.addLine("rpm > 1000");
                updateTelemetry(telemetry);

                telemetry.addLine("scoop up");
                updateTelemetry(telemetry);
                scoop.setPosition(scoopUpPos);

                if (elapsed >= 1000) {
                    telemetry.addLine("scoop down");
                    updateTelemetry(telemetry);
                    scoop.setPosition(scoopDownPos);

                    flywheel.setVelocity(0);

                    // false stops action rerun
                    return false;

                }
            }
            return true;
            // overall, the action powers the flywheel until it surpasses 1000 tps or whatever velocity unit setVelocity() takes, then powers it off
        }
    }

    private Flywheel flywheel;
    // private Servo scoop; alr defined

    public Action flywheelCycle() {
        return new FlywheelCycle(flywheel, scoop);
    }

    // intake wheel in
    public class intakeIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(intakePow);
            return false;
        }
    }

    public Action intakeIn() {
        return new intakeIn();
    }

    // transition wheel in
    public class transIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transition.setPower(transPow);
            return false;
        }
    }

    public Action transIn() {
        return new transIn();
    }

    // run op mode
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Define how the hub is mounted on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);

        // instantiate your MecanumDrive at a particular pose.
        // note: CHANGE POSE BASED ON REAL POSE !!!!!!
        Pose2d initialPose = new Pose2d(5, 5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a flywheel instance
        Flywheel flywheel = new Flywheel(hardwareMap);
        // make a transition instance
        transition transition = new transition(hardwareMap);
        // make an intake instance
        intake intake = new intake(hardwareMap);
        // make a scoop instance
        scoop scoop = new scoop(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder test = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(10, 10))
                .waitSeconds(2)
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder trajectoryActionTwo = drive.actionBuilder(initialPose)
                .lineToY(61)
                .waitSeconds(3)
                .lineToY(11.8)
                .turn(Math.toRadians(180));


        while (!isStopRequested() && !opModeIsActive()) {
            // int position = visionOutputPosition;
            telemetry.addLine("initialized!");
            telemetry.update();
        }

        telemetry.addLine("In Starting Position");
        telemetry.update();
        waitForStart();

        Action trajectoryActionChosen;
        trajectoryActionChosen = test.build();
        Action trajectoryAction2;
        trajectoryAction2 = trajectoryActionTwo.build();


        Actions.runBlocking(
            new SequentialAction(
                trajectoryActionChosen, // add a comma here when u uncomment the next line, the comma should go on all but the last one
                flywheelCycle(),
                transIn()
                // flywheelCycle(),
                // transIn(),
                // flywheelCycle()
            )
        );
    }

}


