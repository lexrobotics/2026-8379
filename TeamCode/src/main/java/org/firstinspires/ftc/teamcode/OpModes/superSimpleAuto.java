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

    double intakePow = 0.5;
    double transPow = 0.5;
    private double flywheelPow = 0.0;

    //common position declarations
    double scoopDownPos = 0.6;
    double scoopUpPos = 0.0;
    double rampClosePos = 0.25;
    double rampMidPos = 0.5;
    double rampFarPos = 0.7;

    double TPR = 100.8;
    double rpm;
    double TPS;

    double targetRPMFar = 1400.0;
    double targetRPMNear = 670.0;
    double targetRPMMid = 1000.0;
    double targetFarTPS = (targetRPMFar*TPR)/60;
    double targetNearTPS = (targetRPMNear)*TPR/60;
    double targetMidTPS = (targetRPMMid)*TPR/60;



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

        //Actual code, not rr-based
        waitForStart();
        ramp.setPosition(rampClosePos);

        back(500);
        HALT();
        sharpshooter();
        //sharpshooter();
        //sharpshooter();



        /*
        //constants
        double shootX = -49.0;
        double shootY = -40.0;
        double shootAngle = Math.toRadians(135);
        double lineupY = -11.0;
        double inOneY = -45.0;
        double inOneX = -12.0;
        double inTwoX = 12.0;
        double inTwoY = -50.0;

        Pose2d beginPose = new Pose2d(-49, -49, Math.toRadians(-125));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        if (isStopRequested()) return;

        drive.localizer.setPose(beginPose);
        drive.updatePoseEstimate();


        waitForStart();

        //creating some sort of path
        Action pewpewpew = drive.actionBuilder(beginPose)
                //.stopAndAdd(new HungryHippo()) //intake and trans always on
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-40, -40, Math.toRadians(-125)), Math.atan2(9.0, 9.0))
                //.strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle)//getting into pos
                .setReversed(false)
                //.stopAndAdd(new MichaelJordan())
                .build();

        Actions.runBlocking(new SequentialAction(pewpewpew));*/
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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servo funcs
        scoop = hardwareMap.get(Servo.class, "scoop");
        ramp = hardwareMap.get(Servo.class, "ramp");
        intake = hardwareMap.get(CRServo.class, "intake");
        transition = hardwareMap.get(CRServo.class, "transition");
    }


    private void back(double time){

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < time) {
            leftFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(0.5);
            telemetry.update();
        }
    }

    private void HALT(){
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }

    private void sharpshooter() {
        ElapsedTime timer = new ElapsedTime();
        int shot = 0;

        while (timer.milliseconds() < 5000) {
            flywheel.setVelocity(targetNearTPS);

            TPS = Math.abs(flywheel.getVelocity());
            rpm = Math.abs((TPS / TPR) * 60.0);

            telemetry.clear(); // to avoid clogging up drive station
            telemetry.addLine("flywheel cycle - automated. current RPM: " + rpm + "\n \n current TPS: " + TPS + "\nTargetTPS: " + targetNearTPS);

            if (Math.abs(TPS - targetNearTPS) < 50) {
                intake.setPower(1);
                transPow = 1;
                transition.setPower(transPow);//so any ball in the transition doesn't block the ball in the scoop
                Wait(250);
                transition.setPower(0);
                // actually shooting the ball w scoop
                scoop.setPosition(scoopUpPos);
                Wait(250);
                scoop.setPosition(scoopDownPos);

                Wait(100);
                transPow = -1;
                transition.setPower(transPow); //loads next ball
                //sleep(3000); //so the cycle doesn't repeat itself too fast
                flywheel.setVelocity(0); // stops flywheel
                Wait(1500);
                transition.setPower(0);
                intake.setPower(0);
                shot += 1;
            } else if (TPS - targetNearTPS > 0) {
                // flywheel too fast
                flywheel.setVelocity(0);
                Wait(500);
                flywheel.setVelocity(targetNearTPS);
            } else {
                // flywheel too slow, nothing needed
                flywheel.setVelocity(targetNearTPS);
            }
        }
    }



    private void Wait(double milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            telemetry.update();
        }
    }
}
