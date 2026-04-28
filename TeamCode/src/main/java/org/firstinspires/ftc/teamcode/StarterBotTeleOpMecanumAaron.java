package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    double launcherTargetVelocity = 1500;
    final double LAUNCHER_MIN_VELOCITY = 1750;
    double kOffset = 0; 
    
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotor intake = null;

    private static final double BLUE_GOAL_X = 14.5;
    private static final double BLUE_GOAL_Y = 129.5;
    private static final double RED_GOAL_X = 130;
    private static final double RED_GOAL_Y = 130;

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

        localizer = new PinpointLocalizer(hardwareMap, 1.0, new Pose2d(0, 0, 0));

        telemetry.addData(">", "Robot Ready. Press Play.");
    }

    @Override
    public void loop() {
        // Update localizer and get robot status
        PoseVelocity2d currentVelocity = localizer.update();
        Pose2d currentPose = localizer.getPose();

        // Calculate distances to goals
        double distToBlue = Math.hypot(BLUE_GOAL_X - currentPose.position.x, BLUE_GOAL_Y - currentPose.position.y);
        double distToRed = Math.hypot(RED_GOAL_X - currentPose.position.x, RED_GOAL_Y - currentPose.position.y);

        // Drive controls (Mecanum)
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = (drive + strafe + twist) / 1.5;
        wheelSpeeds[1] = (drive - strafe - twist) / 1.5;
        wheelSpeeds[2] = (drive - strafe + twist) / 1.5;
        wheelSpeeds[3] = (drive + strafe - twist) / 1.5;

        leftFrontDrive.setPower(wheelSpeeds[0]);
        rightFrontDrive.setPower(wheelSpeeds[1]);
        leftBackDrive.setPower(wheelSpeeds[2]);
        rightBackDrive.setPower(wheelSpeeds[3]);

        // Intake control (using triggers)
        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        // Launcher velocity adjustment (Manual)
        if (gamepad2.dpad_up && !lastDpadUp) {
            launcherTargetVelocity += 100;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            launcherTargetVelocity -= 100;
        }
        lastDpadDown = gamepad2.dpad_down;

        // Auto-Aim with cubic regression
        if (gamepad2.y) {
            launcherTargetVelocity = velocityFromDistance(distToRed) + kOffset;
        } else if (gamepad2.b) {
            launcherTargetVelocity = 0;
            launcher.setVelocity(0);
        }

        // Launcher State Machine
        switch (currentLaunchState) {
            case IDLE:
                if (gamepad2.a || gamepad2.right_bumper) {
                    currentLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(launcherTargetVelocity);
                // Check if up to speed (both minimum and target percentage)
                if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY && launcher.getVelocity() >= launcherTargetVelocity * 0.95) {
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
                    // If button is released, stop launcher and reset
                    if (!gamepad2.a && !gamepad2.right_bumper) {
                        launcher.setVelocity(0);
                        currentLaunchState = LaunchState.IDLE;
                    } else {
                        // If still holding, go back to spin up for next shot
                        currentLaunchState = LaunchState.SPIN_UP;
                    }
                }
                break;
        }

        // Telemetry
        telemetry.addData("State", currentLaunchState);
        telemetry.addData("Target Velocity", launcherTargetVelocity);
        telemetry.addData("Actual Velocity", launcher.getVelocity());
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f)", currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.addData("Velocity", "(%.1f, %.1f, %.1f)", currentVelocity.linearVel.x, currentVelocity.linearVel.y, Math.toDegrees(currentVelocity.angVel));
        telemetry.addData("Red Dist", distToRed);
        telemetry.update();
    }

    @Override
    public void stop() {}

    double velocityFromDistance(double x) {
        // Cubic regression formula: y = -0.000810659x^3 + 0.216733x^2 - 13.70732x + 1783.11384
        return (-0.000810659 * Math.pow(x, 3)) + (0.216733 * Math.pow(x, 2)) - (13.70732 * x) + 1783.11384;
    }
}
