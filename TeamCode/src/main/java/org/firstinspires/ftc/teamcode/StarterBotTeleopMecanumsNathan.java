package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*h
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopMecanumsNathan", group = "StarterBot")
//@Disabled
public class StarterBotTeleopMecanumsNathan extends OpMode {
    final double FEED_TIME_SECONDS = 0.40; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 5250.0;
    private static final double BLUE_GOAL_X = 14.5;
    private static final double BLUE_GOAL_Y = 129.5;
    private static final double RED_GOAL_X = 130;
    private static final double RED_GOAL_Y = 130;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */


    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotor intake = null;
    private PinpointLocalizer localizer = null;
    private Pose2d initialRobotPose = new Pose2d(14.5, 129.5, Math.toRadians(-45));
    private static final double PINPOINT_IN_PER_TICK = 0.0019684344326;
    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double LAUNCHER_TARGET_VELOCITY = 1750;
    double LAUNCHER_MIN_VELOCITY = 1750;
    double INCREASE_VALUE = 10;
    private static final double kOffset = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotor.class,"intake");


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        localizer = new PinpointLocalizer(hardwareMap, PINPOINT_IN_PER_TICK,initialRobotPose);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Pose", "(%.2f, %.2f, %.2f rad)", initialRobotPose.position.x, initialRobotPose.position.y, initialRobotPose.heading.toDouble());
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        /*
        while (gamepad2.dpadUpWasPressed()) {
            LAUNCHER_MIN_VELOCITY = LAUNCHER_MIN_VELOCITY + INCREASE_VALUE;
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_TARGET_VELOCITY + INCREASE_VALUE;
            if (gamepad2.dpadUpWasPressed()) {
                new SleepAction(0.3);
            }
        }
        // while (gamepad2.dpadDownWasPressed()) {
            LAUNCHER_MIN_VELOCITY = LAUNCHER_MIN_VELOCITY - INCREASE_VALUE;
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_TARGET_VELOCITY - INCREASE_VALUE;
            if (gamepad2.dpadDownWasPressed()) {
                new SleepAction(0.3);
            }
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad2.rightBumperWasPressed());

        PoseVelocity2d currentVelocity = localizer.update();
        Pose2d currentPose = localizer.getPose();
        //Distance to BLUE goal
        double distToBlue = Math.hypot(BLUE_GOAL_X - currentPose.position.x, BLUE_GOAL_Y - currentPose.position.y);
        //Distance to BLUE goal
        double distToRed = Math.hypot(RED_GOAL_X - currentPose.position.x, RED_GOAL_Y - currentPose.position.y);
        // intake test

        if (gamepad2.y) {
            LAUNCHER_TARGET_VELOCITY = velocityFromDistance(distToBlue) + kOffset;
            LAUNCHER_MIN_VELOCITY = velocityFromDistance(distToBlue) + kOffset;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("launchSpeedMIN",LAUNCHER_MIN_VELOCITY);
        telemetry.addData("launchSpeedTARGET",LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Pose", "%.1f, %.1f, %.1f", currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.addData("Velocity", "%.1f, %.1f, %.1f", currentVelocity.linearVel.x, currentVelocity.linearVel.y, Math.toDegrees(currentVelocity.angVel));
        telemetry.addData("Blue goal distance", distToBlue);
        telemetry.addData("Red goal distance", distToRed);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / 2.25;
        rightFrontPower = (forward - strafe - rotate) / 2.25;
        leftBackPower = (forward - strafe + rotate) / 2.25;
        rightBackPower = (forward + strafe - rotate) / 2.25 ;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    intake.setPower(0);
                }
                if (gamepad2.left_bumper) {
                intake.setPower(1);
            } else if (gamepad2.leftBumperWasReleased()) {
                intake.setPower(0);
            }
                if(gamepad2.xWasPressed()) {
                    intake.setPower(-1);
                } else if (gamepad2.xWasReleased()) {
                    intake.setPower(0);
                }

                break;
            case SPIN_UP:
                intake.setPower(0);
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
    double velocityFromDistance(double x) {
        //Only clamp minimum (no upper clamp)
        x = Math.max(18, x);

        return -0.000744119 * x * x * x
                +0.228351 * x * x
                -15.52643 * x
                +1716.40744;
    }
}
