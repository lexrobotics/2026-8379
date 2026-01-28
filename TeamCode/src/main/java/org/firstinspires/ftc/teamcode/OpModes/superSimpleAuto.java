package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ftc.Actions;




@Config
@Autonomous(name = "Q2simple", group = "Auto")
public class superSimpleAuto extends LinearOpMode {
    //hardware stuff
    CRServo intake;
    CRServo transition;
    Servo scoop;
    Servo ramp;
    DcMotorEx flywheel;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor rightFront;



    public class MichaelJordan implements InstantFunction{

        public void run(){
//copy and paste the flywheel code
        }
    }

    public class HungryHippo implements InstantFunction {

        public void run(){
//have the intake and trans be on here
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //hardware retrival
        setup();

        //constants
        double shootX = -38.0;
        double shootY = -40.0;
        double shootAngle = Math.toRadians(135);
        double lineupY = -11.0;
        double inOneY = -45.0;
        double inOneX = -12.0;
        double inTwoX = 12.0;
        double inTwoY = -50.0;

        Pose2d beginPose = new Pose2d(-49, -49, Math.toRadians(120));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (isStopRequested()) return;

        drive.localizer.setPose(beginPose);
        drive.updatePoseEstimate();


        waitForStart();

        //creating some sort of path
        Action pewpewpew = drive.actionBuilder(beginPose)
                //.stopAndAdd(new HungryHippo()) //intake and trans always on
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle)//getting into pos
                //.stopAndAdd(new MichaelJordan())
                .build();

        Actions.runBlocking(new SequentialAction(pewpewpew));
    }

    private void setup() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        //servo funcs
        scoop = hardwareMap.get(Servo.class, "scoop");
        ramp = hardwareMap.get(Servo.class, "ramp");
        intake = hardwareMap.get(CRServo.class, "intake");
        transition = hardwareMap.get(CRServo.class, "transition");
    }
}
