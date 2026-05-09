package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleOpMecanumAaron", group = "StarterBot")
public class StarterBotTeleOpMecanumAaron extends OpMode {
    // Constants for feeder and launcher
    final double FEED_TIME_SECONDS = 0.40;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 5250.0;
    final double LAUNCHER_MIN_VELOCITY = 1750;

    // Goal Coordinates
    private static final double RED_GOAL_X = 130.0;
    private static final double RED_GOAL_Y = 130.0;
    private static final double BLUE_GOAL_X = 14.5;
    private static final double BLUE_GOAL_Y = 129.5;
    private static final double Kturn = 1.5;

    // State variables
    double launcherTargetVelocity = 4000;
    double kOffset = 0; 
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    double driverTurn = 0.0;

    // Hardware members
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotor intake = null;

    private PinpointLocalizer localizer = null;
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        FEED,
        STOP_FEED
    }

    private LaunchState currentLaunchState = LaunchState.IDLE;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25, 0, 7, 13.5));

        localizer = new PinpointLocalizer(hardwareMap, 1.0, new Pose2d(72, 72, 0));

        localizer.driver.resetPosAndIMU();
        localizer.setPose(new Pose2d(72, 72, 0));

        telemetry.addData(">", "Robot Ready. Press Play.");
    }

    @Override
    public void loop() {
        // 1. Update Position
        PoseVelocity2d currentVelocity = localizer.update();
        Pose2d currentPose = localizer.getPose();

        // 2. Drive Logic (Mecanum)
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        
        // Auto-turn to blue goal if LB is held
        if (gamepad1.left_bumper) {
            driverTurn = spintoBlue(currentPose);
        } else {
            driverTurn = gamepad1.right_stick_x;
        }
        telemetry.addData("driverTurn", driverTurn);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = (drive + strafe + driverTurn) / 1.5;
        wheelSpeeds[1] = (drive - strafe - driverTurn) / 1.5;
        wheelSpeeds[2] = (drive - strafe + driverTurn) / 1.5;
        wheelSpeeds[3] = (drive + strafe - driverTurn) / 1.5;
        telemetry.addData("wheelSpeeds", wheelSpeeds[0]);
        telemetry.addData("wheelSpeeds", wheelSpeeds[1]);
        telemetry.addData("wheelSpeeds", wheelSpeeds[2]);
        telemetry.addData("wheelSpeeds", wheelSpeeds[3]);

        leftFrontDrive.setPower(wheelSpeeds[0]);
        rightFrontDrive.setPower(wheelSpeeds[1]);
        leftBackDrive.setPower(wheelSpeeds[2]);
        rightBackDrive.setPower(wheelSpeeds[3]);

        // 3. Intake Logic
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        // 4. Flywheel Speed Adjustment
        if (gamepad2.dpad_up && !lastDpadUp) {
            launcherTargetVelocity += 100;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            launcherTargetVelocity -= 100;
        }
        lastDpadDown = gamepad2.dpad_down;

        // Auto-Aim Logic (Y Button)
        double distToRed = Math.hypot(RED_GOAL_X - currentPose.position.x, RED_GOAL_Y - currentPose.position.y);
        double distToBlue = Math.hypot(BLUE_GOAL_X - currentPose.position.x, BLUE_GOAL_Y - currentPose.position.y);
        
        if (gamepad2.y) {
            launcherTargetVelocity = velocityFromDistance(distToRed) + kOffset;
        } else if (gamepad2.b) {
            launcherTargetVelocity = 0;
            launcher.setVelocity(0);
        }

        // 5. Launcher State Machine
        switch (currentLaunchState) {
            case IDLE:
                if (gamepad2.a || gamepad2.right_bumper) {
                    currentLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(launcherTargetVelocity);
                if (Math.abs(launcher.getVelocity()) >= Math.abs(launcherTargetVelocity) * 0.90 && Math.abs(launcherTargetVelocity) > 100) {
                    currentLaunchState = LaunchState.FEED;
                }
                break;
            case FEED:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                currentLaunchState = LaunchState.STOP_FEED;
                break;
            case STOP_FEED:
                if (feederTimer.seconds() >= FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    if (!gamepad2.a && !gamepad2.right_bumper) {
                        launcher.setVelocity(0);
                        currentLaunchState = LaunchState.IDLE;
                    } else {
                        currentLaunchState = LaunchState.SPIN_UP;
                    }
                }
                break;
        }

        // 6. Telemetry
        /*telemetry.addData("State", currentLaunchState);
        telemetry.addData("Target Velocity", launcherTargetVelocity);
        telemetry.addData("Actual Velocity", launcher.getVelocity());
        telemetry.addData("Pose", "%.1f, %.1f, %.1f", currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.addData("Velocity", "%.1f, %.1f, %.1f", currentVelocity.linearVel.x, currentVelocity.linearVel.y, Math.toDegrees(currentVelocity.angVel));
        telemetry.addData("Red Dist", distToRed);
        telemetry.addData("Blue Dist", distToBlue);
        telemetry.update();*/
    }

    @Override
    public void stop() {
    }

    // Helper Methods
    double velocityFromDistance(double x) {
        return (-0.000810659 * Math.pow(x, 3)) + (0.216733 * Math.pow(x, 2)) - (13.70732 * x) + 1783.11384;
    }

    double spintoBlue(Pose2d pose2d) {
        double robotX = pose2d.position.x;
        double robotY = pose2d.position.y;
        double robotHeading = pose2d.heading.toDouble();

        double dx = BLUE_GOAL_X - robotX;
        double dy = BLUE_GOAL_Y - robotY;

        telemetry.addData("robotX", robotX);
        telemetry.addData("robotY", robotY);
        telemetry.addData("dx", dx);
        telemetry.addData("dy", dy);
        telemetry.addData("robotHeading", robotHeading);

        double targetAngle = Math.atan2(dy, dx);
        telemetry.addData("targetAngle", targetAngle);
        double angleError = targetAngle - robotHeading;
        telemetry.addData("angleError", angleError);

        // Wrap angle error to [-pi, pi]
        angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
        telemetry.addData("wraped angleError", angleError);

        telemetry.addData("return", -Kturn * angleError);

        return -Kturn * angleError;
    }
}
