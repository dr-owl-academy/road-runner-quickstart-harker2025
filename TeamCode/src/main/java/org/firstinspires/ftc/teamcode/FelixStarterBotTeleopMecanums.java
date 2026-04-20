

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



@TeleOp(name = "FelixStarterBotTeleopMecanum", group = "StarterBot")
//@Disabled
public class FelixStarterBotTeleopMecanums extends OpMode {
    final double FEED_TIME_SECONDS = 0.40; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = -6000.0;
    final double BLUE_GOAL_X = 14.5;
    final double BLUE_GOAL_Y = 129.5;
    final double RED_GOAL_X = 130;
    final double RED_GOAL_Y = 130;


     double LAUNCHER_TARGET_VELOCITY = 1125;
     double LAUNCHER_MIN_VELOCITY = 1075;

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
    private Pose2d initialRobotPose = new Pose2d(9.3, 48, Math.toRadians(90));
    private static final double PINPOINT_IN_PER_TICK = 0.0019684344326;

    ElapsedTime feederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState currentLaunchState;


    // Set up a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init(){
        currentLaunchState = LaunchState.IDLE;

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
        intake = hardwareMap.get(DcMotor.class, "intake");

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
        localizer = new PinpointLocalizer(hardwareMap, PINPOINT_IN_PER_TICK, initialRobotPose);

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if (gamepad2.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }
        while (gamepad2.dpadUpWasPressed()) {
            LAUNCHER_MIN_VELOCITY = LAUNCHER_MIN_VELOCITY + 10;
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_TARGET_VELOCITY + 10;

        }
        while (gamepad2.dpadDownWasPressed()) {
            LAUNCHER_MIN_VELOCITY = LAUNCHER_MIN_VELOCITY - 10;
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_TARGET_VELOCITY - 10;

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


        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", currentLaunchState);
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
        switch (currentLaunchState) {
            case IDLE:
                if (shotRequested) {
                    currentLaunchState = LaunchState.SPIN_UP;
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
                    currentLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                currentLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    currentLaunchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
    double velocityFromDistance(double x) {
        //Only clamp minimum (no upper clamp)
        x = Math.max(18, x);

        return 0.000556157 * x * x* x
                -0.174432 * x *x
                +22.77848*x
                +731.43596;
    }
}


