/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.OpModes;

// robot connection commands:
// (navigate to sdk)
//  ./platform-tools/adb connect 192.168.43.1:5555
// android studio should automatically connect so u can run

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
/* Scrimmage teleop :)
* imu/field relative driving is currently disabled hence the commented lines
* also there is no defined imu in tha hardware map */
@TeleOp(name = "ScrimmageTeleOp", group = "TeleOp")
//@Disabled
public class ScrimmageTeleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // declaring chassis motors (add more motors/servos later !!)
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, flywheel; // not sure if the flywheel should be a dcmotor but im pretty sure its dcmotorex
    // note for declaration: DcMotor/DcMotorEx, Servo/CRServo (continuous rotation servo)
    private Servo scoop, ramp;
    private CRServo intake, transition;
    public static double POWEROFFSET = 1;

    double intakePow, transitionPow = 0.5;
    double scoopPos, rampPos = 0.0; // initial position
    double flywheelPow;
    double ticksPerRevolution = flywheel.getMotorType().getTicksPerRev();
    boolean scoopUp = false;


    // This declares the IMU needed to get the current direction the robot is facing
    // we arent using it this year but for future reference :)
    //IMU imu;

    // initializes motors, happens when you press the Init button on the drive station
    @Override
    public void init() {
        // getting all the stuff from hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        scoop = hardwareMap.get(Servo.class, "scoop");
        ramp = hardwareMap.get(Servo.class, "ramp");

        intake = hardwareMap.get(CRServo.class, "intake");
        transition = hardwareMap.get(CRServo.class, "transition");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        // setting them to init position
        scoop.setPosition(scoopPos);
        ramp.setPosition(rampPos);

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // for the rpm tracker
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* // running with encoders (might add later)
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);*/

        /* // for driving relative to the field and not the robot
        // for getting the imu into the hardware map: click on the control hub and name it "imu"
        // also make sure the directions are correct!!
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));*/
    }

    // put all teleop code here!! this is called automatically throughout (!!) when the teleop runs
    @Override
    public void loop() {
        // telemetry.addLine("Press A to reset Yaw");
        // telemetry.addLine("Hold left bumper to drive in robot relative");
        // telemetry.addLine("The left joystick sets the robot direction");
        // telemetry.addLine("Moving the right joystick left and right turns the robot");

        // Y: high position (farther)
        // B: middle position
        // A: low position (closer)


        /* If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        // (basically saying "oh this is forward")
        if (gamepad1.a) {
            imu.resetYaw();
        }*/

        driveTrain();

        /*// If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            driveTrain();
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }*/

        // add other code here with if statements !!
        // note for servos: check whether it needs to be 1 or -1!!

        // brings scoop down - BACKUP - click
        if (gamepad2.a) {
            scoopPos = 0.0;
            scoop.setPosition(scoopPos);
        }

        // brings scoop UP - BACKUP - click
        if (gamepad2.x) {
            scoopPos = 0.5;
            scoop.setPosition(scoopPos);
        }

        // OUTTAKE - turn on the flywheel, test if at a certain RPM, bring scoop up, wait a little, bring scoop down - hold
        while (gamepad2.y) {
            flywheel.setVelocity(targetTPS);
            double current = flywheel.getVelocity();

            if (Math.abs(current - targetTPS) < 150) {
                telemetry.addLine("Flywheel ready. 1550 RPM");
                scoop.setPosition(scoopUp);

                while (runtime.seconds() < 3.0) {
                    // telemetry.addLine("Waiting for outtake");
                    //telemetry.removeLine(); // idk i dont want it to flood
                }
                scoopPos = 0.0;
                scoop.setPosition(scoopPos); // bringing scoop back down

                // idk
            }
        }

        // intake wheel position controlled by right joystick - BACKUP - hold
        while (gamepad2.right_stick_y < 0) {
            intakePow = -1.0; // theoretically should spin it in reverse if gamepad goes down
            intake.setPower(intakePow);
        }
        while (gamepad2.right_stick_y > 0) {
            intakePow = 1.0;
            intake.setPower(intakePow);
        }

        // transition wheel position controlled by left joystick - BACKUP - hold
        while (gamepad2.left_stick_y < 0) {
            transitionPow = -1.0; // theoretically should spin it in reverse if gamepad goes down
            transition.setPower(transitionPow);
        }
        while (gamepad2.left_stick_y > 0) {
            transitionPow = 1.0;
            transition.setPower(transitionPow);
        }

        // dpad controls the ramp - click
        // up --> highest position (far aiming spot), left --> middle position (in between spot), down --> low position (close spot)
        // its an if/else if/else if so that clicking multiple buttons at once doesn't confuse it like just in case idk
        if (gamepad2.dpad_up) {
            rampPos = 0.5;
            ramp.setPosition(rampPos);
        } else if (gamepad2.dpad_down) {
            rampPos = 0.0;
            ramp.setPosition(rampPos);
        } else if (gamepad2.dpad_left) {
            rampPos = 0.25;
            ramp.setPosition(rampPos);
        }
    }

    // called during loop()
    private void driveTrain() { // robot-relative driving
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        double leftFrontPower = drive + strafe + rotate;
        double leftRearPower = drive - strafe + rotate;
        double rightFrontPower = drive - strafe - rotate;
        double rightRearPower = drive + strafe - rotate;

        leftFront.setPower(leftFrontPower * POWEROFFSET);
        leftBack.setPower(leftRearPower * POWEROFFSET);
        rightFront.setPower(rightFrontPower * POWEROFFSET);
        rightBack.setPower(rightRearPower * POWEROFFSET);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        //theta = AngleUnit.normalizeRadians(theta -
                //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // not called i think but its similar to driveTrain()
    private void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make maxSpeed lower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // multiply by maxSpeed so that it can be set lower for outreaches
        leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        rightFront.setPower(maxSpeed * (frontRightPower / maxPower));
        leftBack.setPower(maxSpeed * (backLeftPower / maxPower));
        rightBack.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
