package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// robot connection commands:
// (navigate to sdk) connect to robot wifi
// ./platform-tools/adb connect 192.168.43.1:5555
// android studio should automatically connect so u can run

@TeleOp(name = "Qual1Teleop", group = "TeleOp")

public class Qual1Teleop extends LinearOpMode {

    // private ElapsedTime runtime = new ElapsedTime();

    //Name declarations
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

    double TPR = 100.8;
    double rpm;
    double TPS;

    double targetRPMFar = 1400.0;
    double targetRPMNear = 650.0;
    double targetFarTPS = (targetRPMFar*TPR)/60;

    double targetNearTPS = (targetRPMNear)*TPR/60;

    public void runOpMode() {

        //grabs names and assigns stuff form the ds config
        setup();



        //initializing ramp and scoop positions
        ramp.setPosition(rampClosePos);
        scoop.setPosition(scoopDownPos);

        telemetry.addLine("Initialization complete.");
        updateTelemetry(telemetry);

        waitForStart();
        if (isStopRequested()) return;

        //

        while (opModeIsActive()) {

            driveTrain(); //aka d1 (wheels)
            robot(); //aka d2 (robo funcs)

        }

    }

    public void driveTrain() {
        // telemetry.addLine("Drivetrain on");
        updateTelemetry(telemetry);
        //robot-relative driving
        //assigning stick funcs
        double drive = -gamepad1.left_stick_y; // Forward/backward
        double strafe = -gamepad1.left_stick_x; // Left/right
        double rotate = -gamepad1.right_stick_x * 0.75; // Rotation

        // Calculate power for each motor
        double leftFrontPower = drive + strafe + rotate;
        double leftRearPower = -drive + strafe - rotate;
        double rightFrontPower = drive - strafe - rotate;
        double rightRearPower = -drive - strafe + rotate;

        // Apply power to the motors
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightRearPower);
    }

    public void robot(){
        //sets up telemetry so we can call it later
        //Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // flywheel encoder stuff to get RPM
        TPS = flywheel.getVelocity();
        rpm = Math.abs((TPS / TPR) * 60.0);

        flywheel.setPower(0.00);

        // OVERRIDE CODE
        // intake
        if (gamepad2.left_stick_y != 0) {
            if (gamepad2.left_stick_y > 0) {
                intakePow = -1;
            } else {
                intakePow = 1;
            }
            intake.setPower(intakePow);
        } else {
            intakePow = 0;
            intake.setPower(intakePow);
        }

        //Transition
        if (gamepad2.right_stick_y != 0) {
            if (gamepad2.right_stick_y > 0) {
                telemetry.addLine("T-neg power");
                updateTelemetry(telemetry);
                transPow = -1;
                transition.setPower(transPow);
            } else {
                telemetry.addLine("T-pos power");
                updateTelemetry(telemetry);
                transPow = 1;
                transition.setPower(transPow);
            }

        } else {
            transPow = 0;
            transition.setPower(transPow);
        }

        // scoop down
        if (gamepad2.a) {
            telemetry.addLine("scoop down");
            updateTelemetry(telemetry);
            scoop.setPosition(scoopDownPos);
        }

        // scoop up
        if (gamepad2.x) {
            telemetry.addLine("scoop up");
            updateTelemetry(telemetry);
            scoop.setPosition(scoopUpPos);
        }

        // fake outtake cycle (no rpm since this is w/o encoders and I didn't include intake/trans stuff)
        if (gamepad2.y){
            driveTrain();
            // flywheel encoder stuff to get RPM
            TPS = flywheel.getVelocity();
            rpm = Math.abs((TPS / TPR) * 60.0);

            telemetry.addLine("flywheel cycle");
            updateTelemetry(telemetry);
            flywheel.setDirection(DcMotor.Direction.REVERSE);
            //flywheelPow = 0.75; // bc too powerful at 1
            flywheel.setVelocity(targetFarTPS);

            // rpm "tracker" for override
            if (rpm > 1000) {
                telemetry.addLine("RPM > 1000");
                updateTelemetry(telemetry);
            }

            if (gamepad2.a) {
                telemetry.addLine("scoop down");
                updateTelemetry(telemetry);
                scoop.setPosition(scoopDownPos);
            }

            // scoop up
            if (gamepad2.x) {
                telemetry.addLine("scoop up");
                updateTelemetry(telemetry);
                scoop.setPosition(scoopUpPos);

                Wait(1000);

                telemetry.addLine("scoop down");
                updateTelemetry(telemetry);
                scoop.setPosition(scoopDownPos);
            }
        }

        // NORMAL FUNCTIONS

        // full flywheel automated cycle
        /*if (gamepad2.b) {
            //driveTrain(); // so chassis can run while flywheel


            //velocity-based control instead of climbing RPM-based control
            flywheel.setVelocity(targetFarTPS);

            TPS = Math.abs(flywheel.getVelocity());
            rpm = Math.abs((TPS / TPR) * 60.0);

            telemetry.addLine("flywheel cycle - automated. current RPM: " + rpm + "\ncurrent TPS: " + TPS + "\nTargetTPS: " + targetFarTPS);

            if(Math.abs(TPS - targetFarTPS) < 50){
                intake.setPower(1);
                transPow = 1;
                transition.setPower(transPow);//so any ball in the transition doesn'tblocktheball in the scoop
                sleep(250);
                transition.setPower(0);
                scoop.setPosition(scoopUpPos);
                sleep(250);
                scoop.setPosition(scoopDownPos);

                transPow = -1;
                transition.setPower(transPow);//loads next ball
                sleep(3000); //so the cycle doesn't repeat itself too fast
                flywheel.setVelocity(0); // stops flywheel
                sleep(200);
                intake.setPower(0);

            }


        }*/

        // far shoot
        while (gamepad2.dpad_up) {
            driveTrain();
            flywheel.setVelocity(targetFarTPS);

            TPS = Math.abs(flywheel.getVelocity());
            rpm = Math.abs((TPS / TPR) * 60.0);

            telemetry.clear(); // to avoid clogging up drive station
            telemetry.addLine("flywheel cycle - automated. current RPM: " + rpm + "\ncurrent TPS: " + TPS + "\nTargetTPS: " + targetNearTPS);

            if (Math.abs(TPS - targetFarTPS) < 50) {
                intake.setPower(1);
                transPow = 1;
                transition.setPower(transPow);//so any ball in the transition doesn't block the ball in the scoop
                Wait(250);
                transition.setPower(0);
                // actually shooting the ball w scoop
                scoop.setPosition(scoopUpPos);
                Wait(250);
                scoop.setPosition(scoopDownPos);

                transPow = -1;
                transition.setPower(transPow); //loads next ball
                //sleep(3000); //so the cycle doesn't repeat itself too fast
                flywheel.setVelocity(0); // stops flywheel
                Wait(1500);
                transition.setPower(0);
                intake.setPower(0);
            } else if (TPS - targetFarTPS > 0) {
                // flywheel too fast
                flywheel.setVelocity(0);
                Wait(500);
                flywheel.setVelocity(targetFarTPS);
            } else {
                // flywheel too slow, nothing needed
                flywheel.setVelocity(targetFarTPS);
            }
        }

        // nearshoot
        while (gamepad2.dpad_down) {
            driveTrain();
            flywheel.setVelocity(targetNearTPS);

            TPS = Math.abs(flywheel.getVelocity());
            rpm = Math.abs((TPS / TPR) * 60.0);

            telemetry.clear(); // to avoid clogging up drive station
            telemetry.addLine("flywheel cycle - automated. current RPM: " + rpm + "\ncurrent TPS: " + TPS + "\nTargetTPS: " + targetNearTPS);

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

                transPow = -1;
                transition.setPower(transPow); //loads next ball
                //sleep(3000); //so the cycle doesn't repeat itself too fast
                flywheel.setVelocity(0); // stops flywheel
                Wait(1500);
                transition.setPower(0);
                intake.setPower(0);
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

        /* old ramp code
        if (gamepad2.dpad_up) {
            telemetry.addLine("high ramp");
            updateTelemetry(telemetry);
            ramp.setPosition(rampFarPos);
        }
        if (gamepad2.dpad_left) {
            telemetry.addLine("mid ramp");
            updateTelemetry(telemetry);
            ramp.setPosition(rampMidPos);
        }
        if (gamepad2.dpad_down) {
            telemetry.addLine("low ramp");
            updateTelemetry(telemetry);
            ramp.setPosition(rampClosePos);
        }

         */



        //intake+transition
        //if(gamepad2.)
    }

    private void setup() {
        telemetry.addLine("getting the motors");
        updateTelemetry(telemetry);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        //servo funcs
        telemetry.addLine("getting the servos");
        updateTelemetry(telemetry);
        scoop = hardwareMap.get(Servo.class, "scoop");
        ramp = hardwareMap.get(Servo.class, "ramp");
        intake = hardwareMap.get(CRServo.class, "intake");
        transition = hardwareMap.get(CRServo.class, "transition");

    }

    private void Wait(double milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            driveTrain();
            telemetry.update();
        }
    }
}
