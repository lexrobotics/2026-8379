//package org.firstinspires.ftc.teamcode.OpModes;
//
//import androidx.annotation.NonNull;
//
//// RR-specific imports
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//
//// Non-RR imports
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Config
//@Disabled
//@Autonomous(name="Qual2AutoNear", group="Robot")
//public class Qual2AutoNear extends LinearOpMode {
//
//    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
//    private Servo scoop, ramp;
//    private CRServo intake, transition;
//
//    //common power declarations
//    double intakePow = 0.5;
//    double transPow = 0.5;
//    private double flywheelPow = 0.0;
//
//    //common position declarations
//    double scoopDownPos = 0.6;
//    double scoopUpPos = 0.0;
//    double rampClosePos = 0.0;
//    double rampMidPos = 0.3;
//    double rampFarPos = 0.5;
//
//    double TPR = 104;
//    double rpm;
//    double TPS;
//    double targetRPMFar = 1400.0;
//    double targetRPMNear = 670.0;
//    double targetRPMMid = 1000.0;
//    double targetFarTPS = (targetRPMFar*TPR)/60;
//    double targetNearTPS = (targetRPMNear)*TPR/60;
//    double targetMidTPS = (targetRPMMid)*TPR/60;
//
//    // non chassis declarations: flywheel, intake, transition, scoop
//    public class Flywheel {
//        private DcMotorEx flywheel;
//
//        public Flywheel (HardwareMap hardwareMap) {
//            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
//            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            flywheel.setDirection(DcMotorEx.Direction.FORWARD);
//        }
//
//        public void setVelocity(double power) {
//            flywheel.setVelocity(power);
//        }
//
//        public double getVelocity() {
//            return flywheel.getVelocity();
//        }
//
//        public void stop() {
//            flywheel.setPower(0.0);
//        }
//
//    }
//
//    public class Intake {
//        private CRServo intake;
//
//        public Intake (HardwareMap hardwareMap) {
//            intake = hardwareMap.get(CRServo.class, "intake");
//        }
//
//        public void setPower(double power) {
//            intake.setPower(power);
//        }
//    }
//
//    public class Transition {
//        private CRServo transition;
//
//        public Transition (HardwareMap hardwareMap) {
//            transition = hardwareMap.get(CRServo.class, "transition");
//        }
//
//        public void setPower(double power) {
//            transition.setPower(power);
//        }
//    }
//
//    public class Scoop {
//        private Servo scoop;
//
//        public Scoop (HardwareMap hardwareMap) {
//            scoop = hardwareMap.get(Servo.class, "scoop");
//        }
//
//        public void setPosition (double pos) {
//            scoop.setPosition(pos);
//        }
//    }
//
//    public class Ramp {
//        private Servo ramp;
//
//        public Ramp (HardwareMap hardwareMap) {
//            ramp = hardwareMap.get(Servo.class, "ramp");
//        }
//
//        public void setPosition (double pos) {
//            ramp.setPosition(pos);
//        }
//    }
//
//    public boolean FlywheelCycle implements Action (Flywheel flywheel, Scoop scoop, Ramp ramp, Transition transition, double targetTPS, double targetRPM) {
//
//        public FlywheelCycle(Flywheel flywheel, Servo scoop) {
//            this.flywheel = flywheel;
//            this.scoop = scoop;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//
//            // powers on motor, if it is not on
//            if (!flywheelInitialized) {
//                flywheel.setVelocity(0);
//                flywheelInitialized = true;
//                startTime = System.currentTimeMillis(); // <-- define start time here
//            }
//
//            long elapsed = System.currentTimeMillis() - startTime;
//
//            // checks flywheel's current position
//            double rpm = Math.abs((flywheel.getVelocity() / 104) * 60.0);
//            flywheel.setVelocity(targetTPS);
//            if (rpm < targetRPM) {
//                // true causes the action to rerun, so with a velocity of < 1000 it keeps going
//                return true;
//            } else {
//                telemetry.addLine("rpm > target");
//                updateTelemetry(telemetry);
//
//                telemetry.addLine("scoop up");
//                updateTelemetry(telemetry);
//                scoop.setPosition(scoopUpPos);
//
//                if (elapsed >= 500) {
//                    telemetry.addLine("scoop down");
//                    updateTelemetry(telemetry);
//                    scoop.setPosition(scoopDownPos);
//
//                    flywheel.setVelocity(0);
//
//                    // false stops action rerun
//                    return false;
//
//                }
//            }
//            return true;
//            // overall, the action powers the flywheel until it surpasses 1000 tps or whatever velocity unit setVelocity() takes, then powers it off
//        }
//
//        // automated flywheel cycle - action ver
//
////    public class FlywheelCycle implements Action {
////        private Flywheel flywheel;
////        private Servo scoop;
////        private long startTime;
////
////        // checks if the flywheel motor has been powered on
////        private boolean initialized = false;
////
////        public FlywheelCycle(Flywheel flywheel, Servo scoop) {
////            this.flywheel = flywheel;
////            this.scoop = scoop;
////        }
////
////        // actions are formatted via telemetry packets as below
////        @Override
////        public boolean run(@NonNull TelemetryPacket packet) {
////            // powers on motor, if it is not on
////            if (!initialized) {
////                flywheel.setVelocity(0);
////                initialized = true;
////                startTime = System.currentTimeMillis(); // <-- define start time here
////            }
////
////            long elapsed = System.currentTimeMillis() - startTime;
////
////            // checks flywheel's current position
////            double rpm = Math.abs((flywheel.getVelocity()/104) * 60.0);
////            flywheel.setVelocity(1100);
////            packet.put("flywheelVelocity", rpm);
////            if (rpm < 1000.0) {
////                // true causes the action to rerun, so with a velocity of < 1000 it keeps going
////                return true;
////            } else {
////                telemetry.addLine("rpm > 1000");
////                updateTelemetry(telemetry);
////
////                telemetry.addLine("scoop up");
////                updateTelemetry(telemetry);
////                scoop.setPosition(scoopUpPos);
////
////                if (elapsed >= 1000) {
////                    telemetry.addLine("scoop down");
////                    updateTelemetry(telemetry);
////                    scoop.setPosition(scoopDownPos);
////
////                    flywheel.setVelocity(0);
////
////                    // false stops action rerun
////                    return false;
////
////                }
////            }
////            return true;
////            // overall, the action powers the flywheel until it surpasses 1000 tps or whatever velocity unit setVelocity() takes, then powers it off
////        }
////    }
//
//    public class Robot {
//        public MecanumDrive drive;
//        ;
//        public Flywheel flywheel;
//        public Transition transition;
//        public Scoop scoop;
//        public Ramp ramp;
//        public Intake intake;
//
//        public Robot(HardwareMap hardwareMap) {
//            // UPDATE POSE WITH REAL POSE
//            drive = new MecanumDrive(hardwareMap, new Pose2d(5, 5, Math.toRadians(90)));
//            flywheel = new Flywheel(hardwareMap);
//            scoop = new Scoop(hardwareMap);
//            ramp = new Ramp(hardwareMap);
//            transition = new Transition(hardwareMap);
//            intake = new Intake(hardwareMap);
//        }
//
//        private boolean flywheelInitialized = false;
//        // private boolean flywheelActive = true;
//        private long startTime;
//
//        public boolean FlywheelCycle implements Action (double targetTPS, double targetRPM) {
//
//            public FlywheelCycle(Flywheel flywheel, Servo scoop) {
//                this.flywheel = flywheel;
//                this.scoop = scoop;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//
//                // powers on motor, if it is not on
//                if (!flywheelInitialized) {
//                    flywheel.setVelocity(0);
//                    flywheelInitialized = true;
//                    startTime = System.currentTimeMillis(); // <-- define start time here
//                }
//
//                long elapsed = System.currentTimeMillis() - startTime;
//
//                // checks flywheel's current position
//                double rpm = Math.abs((flywheel.getVelocity() / 104) * 60.0);
//                flywheel.setVelocity(targetTPS);
//                if (rpm < targetRPM) {
//                    // true causes the action to rerun, so with a velocity of < 1000 it keeps going
//                    return true;
//                } else {
//                    telemetry.addLine("rpm > target");
//                    updateTelemetry(telemetry);
//
//                    telemetry.addLine("scoop up");
//                    updateTelemetry(telemetry);
//                    scoop.setPosition(scoopUpPos);
//
//                    if (elapsed >= 500) {
//                        telemetry.addLine("scoop down");
//                        updateTelemetry(telemetry);
//                        scoop.setPosition(scoopDownPos);
//
//                        flywheel.setVelocity(0);
//
//                        // false stops action rerun
//                        return false;
//
//                    }
//                }
//                return true;
//                // overall, the action powers the flywheel until it surpasses 1000 tps or whatever velocity unit setVelocity() takes, then powers it off
//        }
//
//        public Action flywheelCycleNear() {
//            return new FlywheelCycle(targetNearTPS, targetRPMNear);
//        }
//
//        public Action flywheelCycleFar() {
//            return new FlywheelCycle(targetFarTPS, targetRPMFar);
//        }
//
//
//    }
//
//    //private Flywheel flywheel;
//
////    public Action flywheelCycle() {
////        return new FlywheelCycle(robot.flywheel, scoop);
////    }
//
//    // run op mode
//    public void runOpMode() {
//
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
//        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//
//        SparkFunOTOS myOtos;
//
//        // instantiate your MecanumDrive at a particular pose.
//        // note: CHANGE POSE BASED ON REAL POSE !!!!!!
//        Pose2d initialPose = new Pose2d(5, 5, Math.toRadians(90));
//        // make a robot instance
//        Robot robot = new Robot(hardwareMap);;
//
//
//        // actionBuilder builds from the drive steps passed to it
//        TrajectoryActionBuilder test = robot.drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(10, 10))
//                .waitSeconds(2)
//                .turn(Math.toRadians(180));
//
//        TrajectoryActionBuilder trajectoryActionTwo = robot.drive.actionBuilder(initialPose)
//                .lineToY(61)
//                .waitSeconds(3)
//                .lineToY(11.8)
//                .turn(Math.toRadians(180));
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            // int position = visionOutputPosition;
//            telemetry.addLine("initialized!");
//            telemetry.update();
//        }
//
//        telemetry.addLine("In Starting Position");
//        telemetry.update();
//        waitForStart();
//
//        Action trajectoryActionChosen;
//        trajectoryActionChosen = test.build();
//        Action trajectoryAction2;
//        trajectoryAction2 = trajectoryActionTwo.build();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen, // add a comma here when u uncomment the next line, the comma should go on all but the last one
//                        robot.flywheelCycleNear()
//                )
//        );
//    }
//
//}
//
//
