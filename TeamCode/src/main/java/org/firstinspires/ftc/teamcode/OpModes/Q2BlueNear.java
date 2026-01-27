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
@Autonomous(name = "Q2BlueNear", group = "Auto")
public class Q2BlueNear extends LinearOpMode {
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
        double shootAngle = Math.toRadians(240);
        double lineupY = -11.0;
        double inOneY = -45.0;
        double inOneX = -12.0;
        double inTwoX = 12.0;
        double inTwoY = -50.0;

        Pose2d beginPose = new Pose2d(new Vector2d(0, 0), Math. toRadians(120));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d lastPose = beginPose;

        waitForStart();

        //creating some sort of path
        Action pewpewpew = drive.actionBuilder(lastPose)
                //.stopAndAdd(new HungryHippo()) //intake and trans always on
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle)//getting into pos
                //.stopAndAdd(new MichaelJordan())
                .strafeToSplineHeading(new Vector2d(inOneX, lineupY), Math.toRadians(270.00))   //lineup to eat
                .strafeToSplineHeading(new Vector2d(inOneX, inOneY), Math.toRadians(270.00)) //eat
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle) //shooting second
                //.stopAndAdd(new MichaelJordan())
                .strafeToSplineHeading(new Vector2d(inTwoX, lineupY),  Math.toRadians(270.00)) //lineup to eat
                .strafeToSplineHeading(new Vector2d(inTwoX, inTwoY), Math.toRadians(270)) //eat
                .strafeToSplineHeading(new Vector2d(inTwoX, lineupY),  Math.toRadians(270.00)) //back up a tad
                //.stopAndAdd(new MichaelJordan())
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle)//shoot third
                .build();

        Actions.runBlocking(new SequentialAction(pewpewpew));
    }

    private void setup() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        //servo funcs
        scoop = hardwareMap.get(Servo.class, "scoop");
        ramp = hardwareMap.get(Servo.class, "ramp");
        intake = hardwareMap.get(CRServo.class, "intake");
        transition = hardwareMap.get(CRServo.class, "transition");
    }
}
