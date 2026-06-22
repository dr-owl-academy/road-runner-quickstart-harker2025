
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**/
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Adrianauton", group = "StarterBot")
public class Adrianauton extends OpMode {


    // Shooter / intake hardware
    // -----------------------------
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private DcMotor intake;

    // Pedro controls the drivetrain.
    private Follower follower;

    // -----------------------------
    // Field constants
    // -----------------------------
    private static final double RED_GOAL_X = 130.0;
    private static final double RED_GOAL_Y = 130.0;

    private static final Pose START_POSE = new Pose(108.255,132.277 , Math.toRadians(90));

    private static final Pose SHOOT_POSE = new Pose(104.1452282, 102.7378976, Math.toRadians(45));
    private static final Pose START_INTAKE_POSE = new Pose(105.05229056629747,81.95224211585366, Math.toRadians(0));

    private static final Pose INTAKE_POSE = new Pose(127.62830058001263,81.97030073323569 , Math.toRadians(0));

    // -----------------------------
    // Shooter constants
    // -----------------------------
    private static final double FULL_SPEED = 1.0;
    private static final double STOP_SPEED = 0.0;

    private static final double FEED_TIME_SECONDS = 0.20;
    private static final double TIME_BETWEEN_SHOTS_SECONDS = 0.35;

    private static final int PRELOAD_BALL_COUNT = 3;
    //hfgfhgfgfg
    private static final int INTAKED_BALL_COUNT = 3;
    private double redGoalHeading;

    /*
     * CHANGE:
     * Instead of shooting as soon as the flywheel is above 500,
     * wait until the actual velocity is close to the calculated target.
     *
     * If the robot waits too long, increase this number slightly.
     * If the robot still shoots too weakly, decrease this number.
     */
    private static final double SHOOTER_READY_TOLERANCE = 75.0;

    /*
     * CHANGE:
     * This gives the flywheel a minimum amount of time to ramp up,
     * especially before the second shooting sequence.
     *
     * If auto becomes too slow, try 0.50.
     * If shots are still weak, try 0.80 or 1.00.
     */
    private static final double MIN_FLYWHEEL_SPIN_UP_SECONDS = 0.70;

    //use this number to adjust flywheel speed
    private static final double kOffset = 0.0;

    // -----------------------------
    // Path speed constants
    // -----------------------------
    private static final double NORMAL_PATH_POWER = 1;
    private static final double SLOW_INTAKE_PATH_POWER = 0.40;

    // -----------------------------
    // Turn-to-zero constants
    // -----------------------------
    private static final double kTurn = 1.5;
    private static final double MAX_TURN_POWER = 0.35;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);

    private boolean pedroTurnModeStarted = false;

    // -----------------------------
    // Paths
    // -----------------------------
    private PathChain pathToFirstShot;
    private PathChain startPathToIntake;
    private PathChain pathToIntake;
    private PathChain pathBackToShot;


    // -----------------------------
    // State machine variables
    // -----------------------------
    private final ElapsedTime shotTimer = new ElapsedTime();

    /*
     * CHANGE:
     * Separate timer used only for flywheel ramp-up.
     * This makes the second shooting sequence wait properly before feeding balls.
     */
    private final ElapsedTime flywheelTimer = new ElapsedTime();

    private double launcherTargetVelocity = 1500.0;
    private int shotsWanted = 0;
    private int shotsFinished = 0;

    private enum AutoState {
        START_PATH_TO_FIRST_SHOT,
        WAIT_FOR_FIRST_PATH,
        SHOOT_PRELOADS,
        TURN_TO_ZERO_BEFORE_INTAKE,
        START_INTAKE_PATH,
        WAIT_FOR_INTAKE_PATH,
        START_RETURN_PATH,
        WAIT_FOR_RETURN_PATH,
        SHOOT_INTAKED_BALLS,
        DONE
    }

    private enum ShooterState {
        IDLE,
        WAIT_FOR_FLYWHEEL,
        FEED_BALL,
        WAIT_BETWEEN_BALLS,
        DONE
    }

    private AutoState autoState = AutoState.START_PATH_TO_FIRST_SHOT;
    private ShooterState shooterState = ShooterState.IDLE;

    @Override
    public void init() {
        initShooterHardware();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        double redGoalHeading = headingToRedGoal(SHOOT_POSE);

        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", formatPose(START_POSE));
        telemetry.update();
    }



    @Override
    public void start() {
        autoState = AutoState.START_PATH_TO_FIRST_SHOT;
        shooterState = ShooterState.IDLE;
    }

    @Override
    public void loop() {
        follower.update();
        String msg = "";


        switch (autoState) {

            case START_PATH_TO_FIRST_SHOT:
                setLauncherSpeedForPose(SHOOT_POSE);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToFirstShot);

                autoState = AutoState.WAIT_FOR_FIRST_PATH;
                msg = "path to shot 1";
                break;

            /*case WAIT_FOR_FIRST_PATH:
                keepFlywheelReady();

                if (!follower.isBusy()) {
                    beginShooting(PRELOAD_BALL_COUNT);
                    autoState = AutoState.SHOOT_PRELOADS;
                }
                break;*/

            case SHOOT_PRELOADS:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopFeeders();
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.TURN_TO_ZERO_BEFORE_INTAKE;
                }
                break;

            case TURN_TO_ZERO_BEFORE_INTAKE:
                intake.setPower(STOP_SPEED);
                stopFeeders();
                stopFlywheel();

                if (turnToHeadingZeroUsingPedro()) {
                    autoState = AutoState.START_INTAKE_PATH;
                }
                break;

            case START_INTAKE_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();
                msg = "intake";

                follower.setMaxPower(SLOW_INTAKE_PATH_POWER);
                follower.followPath(pathToIntake);

                autoState = AutoState.WAIT_FOR_INTAKE_PATH;
                break;

            case WAIT_FOR_INTAKE_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                if (!follower.isBusy()) {
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.START_RETURN_PATH;
                }
                break;

            case START_RETURN_PATH:
                /*
                 * Flywheel stays off while returning.
                 * This prevents a collected ball from shooting early.
                 */
                stopFlywheel();
                intake.setPower(STOP_SPEED);
                msg = "return";

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot);

                autoState = AutoState.WAIT_FOR_RETURN_PATH;
                break;

            case WAIT_FOR_RETURN_PATH:
                /*
                 * Flywheel still stays off while the robot is moving back.
                 */
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                if (!follower.isBusy()) {
                    /*
                     * CHANGE:
                     * Only now, after the robot has reached the shooting pose,
                     * spin up the flywheel for the second shooting sequence.
                     */
                    setLauncherSpeedForPose(SHOOT_POSE);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS;
                }
                break;

            case SHOOT_INTAKED_BALLS:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                stopEverything();
                break;
        }

        showTelemetry(msg);

    }

    @Override
    public void stop() {
        stopEverything();
    }

    // --------------------------------------------------
    // Setup
    // --------------------------------------------------

    private void initShooterHardware() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotor.class, "intake");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        stopFeeders();
        stopFlywheel();
        intake.setPower(STOP_SPEED);
    }

    private void buildPaths() {
    telemetry.addData("Status","on path");

        pathToFirstShot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(START_POSE.getX(), START_POSE.getY(), START_POSE.getHeading()),
                                new Pose(SHOOT_POSE.getX(), SHOOT_POSE.getY(),redGoalHeading)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        startPathToIntake = follower.pathBuilder()
                .addPath(
                        new BezierLine(SHOOT_POSE,START_INTAKE_POSE)

                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        pathToIntake = follower.pathBuilder()
                .addPath(
                        new BezierLine(START_INTAKE_POSE,INTAKE_POSE)


                )
                .setConstantHeadingInterpolation(0)
                .build();

        pathBackToShot = follower.pathBuilder()

                .addPath(
                        new BezierLine(INTAKE_POSE,SHOOT_POSE)

                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();



        Pose shootPoseFacingRed =
                new Pose((Double) SHOOT_POSE.getX(), (Double) SHOOT_POSE.getY(), redGoalHeading);

        Pose shootPoseHeadingZero =
                new Pose((Double) SHOOT_POSE.getX(), (Double) SHOOT_POSE.getY(), Math.toRadians(0));

        pathToFirstShot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, shootPoseFacingRed))
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(),
                        shootPoseFacingRed.getHeading()
                )
                .build();

        pathToIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathBackToShot = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE, shootPoseFacingRed))
                .setLinearHeadingInterpolation(
                        INTAKE_POSE.getHeading(),
                        shootPoseFacingRed.getHeading()
                )
                .build();


    }

    // --------------------------------------------------
    // Pedro turn-to-zero helper
    // --------------------------------------------------

    private boolean turnToHeadingZeroUsingPedro() {
        startPedroTurnModeIfNeeded();

        double currentHeading = follower.getPose().getHeading();
        double headingError = wrapRadians(currentHeading - Math.toRadians(0));

        if (Math.abs(headingError) <= HEADING_TOLERANCE_RADIANS) {
            stopPedroTurnMode();
            return true;
        }

        double turnPower = Range.clip(
                -kTurn * headingError,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );

        follower.setTeleOpDrive(0, 0, turnPower, true);
        return false;
    }

    private void startPedroTurnModeIfNeeded() {
        if (!pedroTurnModeStarted) {
            follower.startTeleopDrive(true);
            pedroTurnModeStarted = true;
        }
    }

    private void stopPedroTurnMode() {
        follower.setTeleOpDrive(0, 0, 0, true);
        pedroTurnModeStarted = false;
    }

    private double wrapRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    // --------------------------------------------------
    // Shooter state machine
    // --------------------------------------------------

    private void beginShooting(int numberOfBalls) {
        shotsWanted = numberOfBalls;
        shotsFinished = 0;

        shooterState = ShooterState.WAIT_FOR_FLYWHEEL;

        shotTimer.reset();

        /*
         * CHANGE:
         * Start flywheel ramp-up timer whenever a new shooting sequence begins.
         * This is especially important for the second shooting sequence.
         */
        flywheelTimer.reset();
    }

    private boolean updateShooter() {
        intake.setPower(FULL_SPEED);

        switch (shooterState) {

            case WAIT_FOR_FLYWHEEL:
                launcher.setVelocity(launcherTargetVelocity);

                /*
                 * CHANGE:
                 * Do not feed a ball until the flywheel has actually ramped up.
                 */
                if (flywheelIsReady()) {
                    shooterState = ShooterState.FEED_BALL;
                }
                break;

            case FEED_BALL:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);

                shotTimer.reset();
                shooterState = ShooterState.WAIT_BETWEEN_BALLS;
                break;

            case WAIT_BETWEEN_BALLS:
                if (shotTimer.seconds() >= FEED_TIME_SECONDS) {
                    stopFeeders();
                    shotsFinished++;

                    if (shotsFinished >= shotsWanted) {
                        shooterState = ShooterState.DONE;
                    } else {
                        shotTimer.reset();
                        shooterState = ShooterState.IDLE;
                    }
                }
                break;

            case IDLE:
                if (shotTimer.seconds() >= TIME_BETWEEN_SHOTS_SECONDS) {
                    /*
                     * CHANGE:
                     * Reset flywheel timer before checking the next shot.
                     * This gives the flywheel a moment to recover between balls.
                     */
                    flywheelTimer.reset();
                    shooterState = ShooterState.WAIT_FOR_FLYWHEEL;
                }
                break;

            case DONE:
                stopFeeders();
                intake.setPower(STOP_SPEED);
                return true;
        }

        return false;
    }

    private boolean flywheelIsReady() {
        double actualVelocity = launcher.getVelocity();

        boolean waitedLongEnough =
                flywheelTimer.seconds() >= MIN_FLYWHEEL_SPIN_UP_SECONDS;

        boolean closeToTarget =
                actualVelocity >= launcherTargetVelocity - SHOOTER_READY_TOLERANCE;

        return waitedLongEnough && closeToTarget;
    }

    private void stopFeeders() {
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    private void stopFlywheel() {
        launcherTargetVelocity = STOP_SPEED;
        launcher.setVelocity(STOP_SPEED);
        launcher.setPower(STOP_SPEED);
    }

    private void stopEverything() {
        stopFeeders();
        stopFlywheel();
        intake.setPower(STOP_SPEED);

        if (pedroTurnModeStarted) {
            stopPedroTurnMode();
        }
    }

    // --------------------------------------------------
    // Flywheel speed and red-goal aiming
    // --------------------------------------------------

    private void setLauncherSpeedForPose(Pose pose) {
        double distance = distanceToRedGoal(pose);
        launcherTargetVelocity = velocityFromDistance(distance) + kOffset;
        launcher.setVelocity(launcherTargetVelocity);
    }

    private void keepFlywheelReady() {
        setLauncherSpeedForPose(follower.getPose());
    }

    private double velocityFromDistance(double x) {
        x = Math.max(18, x);

        return 0.000764989 * x * x * x
                - 0.216997 * x * x
                + 24.42148 * x
                + 721.27595;
    }

    private double headingToRedGoal(Pose pose) {
        double dx = RED_GOAL_X - pose.getX();
        double dy = RED_GOAL_Y - pose.getY();

        return Math.atan2(dy, dx);
    }

    private double distanceToRedGoal(Pose pose) {
        return Math.hypot(
                RED_GOAL_X - pose.getX(),
                RED_GOAL_Y - pose.getY()
        );
    }

    // --------------------------------------------------
    // Telemetry
    // --------------------------------------------------

    private void showTelemetry(String msg) {
        Pose pose = follower.getPose();

        telemetry.addData("Auto State", autoState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Pose", formatPose(pose));
        telemetry.addData("Distance to Red", "%.1f", distanceToRedGoal(pose));
        telemetry.addData("Target Velocity", "%.1f", launcherTargetVelocity);
        telemetry.addData("Actual Velocity", "%.1f", launcher.getVelocity());
        telemetry.addData("Ready to Shoot", flywheelIsReady());
        telemetry.addData("Flywheel Timer", "%.2f", flywheelTimer.seconds());
        telemetry.addData("Red Heading", "%.1f deg", Math.toDegrees(headingToRedGoal(pose)));
        telemetry.addData("Heading Error to 0", "%.1f deg", Math.toDegrees(wrapRadians(pose.getHeading())));
        telemetry.addData("Shots", "%d / %d", shotsFinished, shotsWanted);
        telemetry.addData("Message", msg);
        telemetry.update();
    }

    private String formatPose(Pose pose) {
        return String.format(
                "(%.1f, %.1f, %.1f deg)",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading())
        );
    }
/*
    private static class Pose {
        public Pose(double v, double v1, double radians) {
        }

        public Object getX() {
            return null;
        }

        public Object getY() {
            return null;
        }

        public double getHeading() {
            return 0;
        }
    }
 */
}
