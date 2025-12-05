package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "MotorTest-hello???", group = "TeleOp")

public class MotorTest extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();

    //Name declarations
    private DcMotor leftFront, leftBack, rightFront, rightBack, flywheel;
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





    public void runOpMode() {
        telemetry.addLine("PLEASE FUCKING WORK THIS TIME.");
        updateTelemetry(telemetry);

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
        telemetry.addLine("Drivetrain on");
        updateTelemetry(telemetry);
        //robot-relative driving
        //assigning stick funcs
        double drive = -gamepad1.left_stick_y; // Forward/backward
        double strafe = -gamepad1.left_stick_x; // Left/right
        double rotate = -gamepad1.right_stick_x; // Rotation

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

        // OVERRIDE CODE
        // intake
        if (gamepad2.left_stick_y != 0) {
            if (gamepad2.left_stick_y > 0) {
                intakePow = -1;
            } else {
                intakePow = 1;
            }
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

        // NORMAL FUNCTIONS
        //fake outtake cycle (no rpm since this is w/o encoders and I didn't include intake/trans stuff)
        while (gamepad2.y){
            telemetry.addLine("flywheel cycle");
            updateTelemetry(telemetry);
            flywheelPow = 1;
            flywheel.setPower(flywheelPow);

            scoop.setPosition(scoopUpPos);
            scoop.setPosition(scoopDownPos);

            flywheel.setPower(0.00);

        }

        // hi
        // gh ramp
        if(gamepad2.dpad_up){
            telemetry.addLine("high ramp");
            updateTelemetry(telemetry);
            ramp.setPosition(rampFarPos);
        }

        // mid ramp
        if(gamepad2.dpad_left){
            telemetry.addLine("mid ramp");
            updateTelemetry(telemetry);
            ramp.setPosition(rampMidPos);
        }

        // close ramp
        if(gamepad2.dpad_down){
            telemetry.addLine("low ramp");
            updateTelemetry(telemetry);
            ramp.setPosition(rampClosePos);
        }
    }

    private void setup() {
        telemetry.addLine("getting the motors");
        updateTelemetry(telemetry);
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        //servo funcs
        telemetry.addLine("getting the servos");
        updateTelemetry(telemetry);
        scoop = hardwareMap.get(Servo.class, "scoop");
        ramp = hardwareMap.get(Servo.class, "ramp");
        intake = hardwareMap.get(CRServo.class, "intake");
        transition = hardwareMap.get(CRServo.class, "transition");

    }
}