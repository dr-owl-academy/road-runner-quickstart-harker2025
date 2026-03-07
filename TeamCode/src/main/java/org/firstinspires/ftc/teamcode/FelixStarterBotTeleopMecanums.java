

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name = "StarterBotTeleopMecanum", group = "StarterBot")
//@Disabled
public class FelixStarterBotTeleopMecanums extends OpMode {
    final double FEED_TIME_SECONDS = 0.40; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = -6000.0;


    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotor intake = null;

    ElapsedTime feederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        INTAKE,
        STOP_INTAKE,
        REVERSE_INTAKE,
        SPIN_UP,
        FEED,
        STOP_FEED
    }

    private LaunchState currentLaunchState;

    // Setup a variable for each drive wheel to save power level for telemetry
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

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
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

        double forward =  (1*gamepad1.right_stick_y);
        double strafe = -1*(gamepad1.right_stick_x);
        double turn = gamepad1.left_stick_x;

        leftFrontPower = ((forward + strafe + turn) / 2);
        rightFrontPower = ((forward - strafe - turn) / 2);
        leftBackPower = ((forward - strafe + turn) / 2);
        rightBackPower = ((forward + strafe - turn) / 2);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }


        /*
         * Now we call our "Launch" function.
         */


        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", currentLaunchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.addData("right stick y", gamepad1.right_stick_y);
        telemetry.addData("left front power", leftFrontPower);
        telemetry.addData("right front power", rightFrontPower);
        telemetry.addData("left back power", leftBackPower);
        telemetry.addData("right back power", rightBackPower);

        switch (currentLaunchState) {
            case IDLE:
                // If the user presses the "a" button, and we are in the IDLE state,
                // we transition to the Intake state.
                if (gamepad2.left_bumper) {
                    currentLaunchState = LaunchState.INTAKE;
                }
                break;
            case INTAKE:
                //The intake motor starts up and doesn't stop unless it is in STOP_INTAKE
                intake.setPower(FULL_SPEED);
                if (gamepad2.a) {
                    currentLaunchState = LaunchState.SPIN_UP;
                }
                if (gamepad2.right_bumper) {
                    currentLaunchState = LaunchState.STOP_INTAKE;
                }
            case STOP_INTAKE:
                //Stops the intake and sets launch state to idle
                intake.setPower(STOP_SPEED);
                currentLaunchState = LaunchState.IDLE;

            case SPIN_UP:
                // In this state, we set the launcher motor to our target velocity.
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // We can check the current velocity of the motor to see if it has reached
                // our minimum velocity. If it has, we transition to the FEED state.
                if(launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    currentLaunchState = LaunchState.FEED;
                }
                if (gamepad2.right_bumper) {
                   intake.setPower(STOP_SPEED);
                }
                break;

            case FEED:
                // In this state, we turn on the feeder servos to push the note into the launcher.
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                // We also reset our timer so we can time how long we run the feeder servos.
                feederTimer.reset();
                if (gamepad2.right_bumper) {
                    intake.setPower(STOP_SPEED);
                }
                currentLaunchState = LaunchState.STOP_FEED;
                break;

            case STOP_FEED:
                // In this state, we wait for the feeder timer to expire.
                if(feederTimer.seconds() >= FEED_TIME_SECONDS) {
                    // When the timer expires, we stop the feeder servos.
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    // We also turn off the launcher motor.
                    launcher.setVelocity(0);
                    // And we transition back to the IDLE state.
                    currentLaunchState = LaunchState.IDLE;
                    if (gamepad2.right_bumper) {
                        intake.setPower(STOP_SPEED);
                    }
                }
                break;
        }


        /*
         * Code to run ONCE after the driver hits STOP
         */


    }

    @Override
    public void stop() {

    }
}