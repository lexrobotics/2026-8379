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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Qual1AutoNear", group="Robot")
public class Qual1AutoNear extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack, flywheel;
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
    private class flywheel {
        private DcMotorEx flywheel;

        public void flywheel (HardwareMap hardwareMap) {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        }
    }

    private class intake {
        private Servo intake;

        public void intake (HardwareMap hardwareMap) {
            intake = hardwareMap.get(Servo.class, "intake");
        }
    }

    private class transition {
        private Servo transition;

        public void transition (HardwareMap hardwareMap) {
            transition = hardwareMap.get(Servo.class, "transition");
        }
    }

    private class scoop {
        private Servo scoop;

        public void scoop (HardwareMap hardwareMap) {
            scoop = hardwareMap.get(Servo.class, "scoop");
        }
    }

    // automated flywheel cycle
    public class flywheelCycle implements Action {
        // checks if the flywheel motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                flywheel.setPower(0.8);
                initialized = true;
            }

            // checks flywheel's current position
            double rpm = Math.abs((flywheel.getVelocity()/104) * 60.0);
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

                sleep(1000);

                telemetry.addLine("scoop down");
                updateTelemetry(telemetry);
                scoop.setPosition(scoopDownPos);

                flywheelPow = 0.0; // stops flywheel automatically
                flywheel.setPower(flywheelPow);

                // false stops action rerun
                return false;
            }
            // overall, the action powers the flywheel until it surpasses 1000 rpm, then powers it off
        }


    }

    public Action flywheelCycle() {
        return new flywheelCycle();
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
        // instantiate your MecanumDrive at a particular pose.
        // note: CHANGE POSE BASED ON REAL POSE !!!!!!
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a flywheel instance
        flywheel flywheel = new flywheel();
        // make a transition instance
        transition transition = new transition();
        // make an intake instance
        intake intake = new intake();
        // make a scoop instance
        scoop scoop = new scoop();

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder test = drive.actionBuilder(initialPose)
                .lineToY(59)
                .waitSeconds(2)
                .lineToX(10)
                .turn(Math.toRadians(180));

        TrajectoryActionBuilder trajectoryActionTwo = drive.actionBuilder(initialPose)
                .lineToY(61)
                .waitSeconds(3)
                .lineToX(11.8)
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
                trajectoryAction2
            )
        );
    }

}


