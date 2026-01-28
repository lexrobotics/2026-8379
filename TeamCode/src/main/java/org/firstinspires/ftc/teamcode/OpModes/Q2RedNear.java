package org.firstinspires.ftc.teamcode.OpModes;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ftc.Actions;




@Config
@Autonomous(name = "Q2redNear", group = "Auto")
public class Q2RedNear extends LinearOpMode {
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
        double shootY = 40.0;
        double shootAngle = Math.toRadians(115);
        double lineupY = 11.0;
        double inOneY = 45.0;
        double inOneX = -12.0;
        double inTwoX = 12.0;
        double inTwoY = 50.0;
        double straightAngle = Math.toRadians(90);

        Pose2d beginPose = new Pose2d(-49, 49, Math. toRadians(125));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d pow = new Pose2d(shootX, shootY, shootAngle);

        waitForStart();

        //creating some sort of path
        Action pewpewpew = drive.actionBuilder(new Pose2d(-49, 49, Math. toRadians(125)))
                //.stopAndAdd(new HungryHippo()) //intake and trans always on
                .setReversed(true)//bkwds
                .splineToSplineHeading(pow, shootAngle)//getting into pos
                //.stopAndAdd(new MichaelJordan())
                .splineToSplineHeading(new Pose2d(inOneX, lineupY, Math.toRadians(90.00)), straightAngle)   //lineup to eat
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(inOneX, inOneY, Math.toRadians(90.00)), straightAngle) //eat
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(shootX, shootY, shootAngle), straightAngle) //shooting second
                //.stopAndAdd(new MichaelJordan())
                .splineToSplineHeading(new Pose2d(inTwoX, lineupY,  Math.toRadians(90.00)),straightAngle)
                //lineup to eat
                .splineToSplineHeading(new Pose2d(inTwoX, inTwoY, Math.toRadians(90)), straightAngle) //eat
                .splineToSplineHeading(new Pose2d(inTwoX, lineupY,  Math.toRadians(90.00)), straightAngle) //back up a tad
                //.stopAndAdd(new MichaelJordan())
                .splineToSplineHeading(new Pose2d(shootX, shootY, shootAngle), straightAngle)//shoot third
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



/*
trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-51.43, 51.64, Math.toRadians(180.00)))
.lineToConstantHeading(new Vector2d(-37.50, 36.00))
.lineToLinearHeading(new Pose2d(-11.57, 23.79, Math.toRadians(90.00)))
.lineToConstantHeading(new Vector2d(-11.79, 51.00))
.lineToLinearHeading(new Pose2d(-37.29, 35.79, Math.toRadians(120.00)))
.lineToLinearHeading(new Pose2d(10.93, 23.57, Math.toRadians(90.00)))
.lineToConstantHeading(new Vector2d(10.93, 62.79))
.lineTo(new Vector2d(10.71, 49.07))
.lineToLinearHeading(new Pose2d(-37.29, 35.79, Math.toRadians(120.00)))
.build();
*/

/*.strafeToConstantHeading(new Vector2d(-37.50, 36.00))   //shooting first
.strafeToLinearHeading(new Vector2d(-11.57, 23.79), Math.toRadians(90.00))   //lineup to eat
.strafeToConstantHeading(new Vector2d(-11.79, 51.00)) //eat
.strafeToLinearHeading(new Vector2d(-37.29, 35.79), Math.toRadians(120.00)) //shooting second
.strafeToLinearHeading(new Vector2d(10.93, 23.57),  Math.toRadians(90.00)) //lineup to eat
.strafeToConstantHeading(new Vector2d(10.93, 62.79)) //eat
.strafeTo(new Vector2d(10.71, 49.07))          //shoot third
.strafeToLinearHeading(new Vector2d(-37.29, 35.79), Math.toRadians(120.00))
.build();*/


