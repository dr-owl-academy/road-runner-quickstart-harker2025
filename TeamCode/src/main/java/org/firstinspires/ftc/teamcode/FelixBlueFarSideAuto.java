package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FelixBlueFarSideAuto", group = "StarterBot")
public class FelixBlueFarSideAuto extends OpMode {

    // -----------------------------
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
    private static final double BLUE_GOAL_X = 14.5;
    private static final double BLUE_GOAL_Y = 129.5;

    private static final Pose START_POSE = new Pose(47.05463182897862, 8.738717339667456, Math.toRadians(90));

    private static final Pose SHOOT_POSE1 = new Pose(58.968000558893394 ,17.052404638815133, Math.toRadians(110));

    private static final Pose SHOOT_POSE2 = new Pose(42.11045125602347, 98.71230941704215, Math.toRadians(135));

    private static final Pose BEFORE_INTAKE1 = new Pose(46.589073634204276, 37.30321159296564, Math.toRadians(180));
    private static final Pose BEFORE_INTAKE2 = new Pose(46.98285948418695, 60.73321471656735, Math.toRadians(180));
    private static final Pose BEFORE_INTAKE3 = new Pose(46.85879759151297, 84.75809287816283, Math.toRadians(180));
    private static final Pose INTAKE_POSE1 = new Pose(23.926017053685545, 37.30321159296564, Math.toRadians(180));
    private static final Pose INTAKE_POSE2 = new Pose(23.403942154044213, 60.73321471656735, Math.toRadians(180));
    private static final Pose INTAKE_POSE3 = new Pose(23.547497741678697, 84.75809287816283, Math.toRadians(180));
    private static final Pose Final_POSE = new Pose(45.17647969298073, 123.85283414832382, Math.toRadians(67));

    // -----------------------------
    // Shooter constants
    // -----------------------------
    private static final double FULL_SPEED = 1.0;
    private static final double STOP_SPEED = 0.0;

    private static final double FEED_TIME_SECONDS = 0.30;
    private static final double TIME_BETWEEN_SHOTS_SECONDS = 1.50;

    private static final int PRELOAD_BALL_COUNT = 2;
    private static final int INTAKED_BALL_COUNT = 2;

    /*
     * CHANGE:
     * Instead of shooting as soon as the flywheel is above 500,
     * wait until the actual velocity is close to the calculated target.
     *
     * If the robot waits too long, increase this number slightly.
     * If the robot still shoots too weakly, decrease this number.
     */
    private static final double SHOOTER_READY_TOLERANCE = 20.0;

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
    private PathChain pathToPreIntake1;
    private PathChain pathToIntake1;
    private PathChain pathBackToShot1;
    private PathChain pathToPreIntake2;
    private PathChain pathToIntake2;
    private PathChain pathBackToShot2;
    private PathChain pathToPreIntake3;
    private PathChain pathToIntake3;
    private PathChain pathBackToShot3;
    private PathChain endsequence;

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
        SHOOT_FIRST_SHOT,
        SHOOT_STOP,
        TURN_TO_ZERO_BEFORE_INTAKE,
        START_INTAKE1_PATH,
        WAIT_FOR_INTAKE1_PATH,
        START_PREINTAKE1_PATH,
        START_RETURN_PATH1,
        WAIT_FOR_RETURN_PATH1,
        SHOOT_INTAKED_BALLS1,
        SHOOT_STOP2,
        TURN_TO_ZERO_BEFORE_INTAKE2,
        START_PREINTAKE2_PATH,
        START_INTAKE2_PATH,
        WAIT_FOR_INTAKE2_PATH,
        START_RETURN_PATH2,
        WAIT_FOR_RETURN_PATH2,
        SHOOT_INTAKED_BALLS2,
        SHOOT_STOP3,
        TURN_TO_ZERO_BEFORE_INTAKE3,
        START_PREINTAKE1_PATH3,
        START_INTAKE3_PATH,
        WAIT_FOR_INTAKE3_PATH,
        START_RETURN_PATH3,
        WAIT_FOR_RETURN_PATH3,
        SHOOT_INTAKED_BALLS3,
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

        switch (autoState) {

            case START_PATH_TO_FIRST_SHOT:
                setLauncherSpeedForPose(SHOOT_POSE1);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToFirstShot);

                autoState = AutoState.SHOOT_FIRST_SHOT;
                break;

            case SHOOT_FIRST_SHOT:

                keepFlywheelReady();

                if (!follower.isBusy()) {

                    beginShooting(PRELOAD_BALL_COUNT);
                    autoState = AutoState.SHOOT_STOP;
                }
                break;

            case SHOOT_STOP:



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
                    autoState = AutoState.START_PREINTAKE1_PATH;
                }
                break;
            case START_PREINTAKE1_PATH:

                stopFlywheel();

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToPreIntake1);

                autoState = AutoState.START_INTAKE1_PATH;
                break;


            case START_INTAKE1_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                follower.setMaxPower(SLOW_INTAKE_PATH_POWER);
                follower.followPath(pathToIntake1);



                autoState = AutoState.WAIT_FOR_INTAKE1_PATH;


                break;

            case WAIT_FOR_INTAKE1_PATH:

                stopFlywheel();

                if (!follower.isBusy()) {
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.START_RETURN_PATH1;
                }
                break;

            case START_RETURN_PATH1:
                /*
                 * Flywheel stays off while returning.
                 * This prevents a collected ball from shooting early.
                 */
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot1);

                autoState = AutoState.WAIT_FOR_RETURN_PATH1;
                break;

            case WAIT_FOR_RETURN_PATH1:
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
                    setLauncherSpeedForPose(SHOOT_POSE1);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS1;
                }
                break;

            case SHOOT_INTAKED_BALLS1:
                FlywheelStart();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.SHOOT_STOP2;
                }
                break;
            case SHOOT_STOP2:


                if (updateShooter()) {
                    stopFeeders();
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.TURN_TO_ZERO_BEFORE_INTAKE2;
                }
                break;

            case TURN_TO_ZERO_BEFORE_INTAKE2:
                intake.setPower(STOP_SPEED);
                stopFeeders();
                stopFlywheel();

                if (turnToHeadingZeroUsingPedro()) {
                    autoState = AutoState.START_PREINTAKE2_PATH;
                }
                break;
            case START_PREINTAKE2_PATH:

                stopFlywheel();

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToPreIntake2);

                autoState = AutoState.START_INTAKE2_PATH;
                break;

            case START_INTAKE2_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                follower.setMaxPower(SLOW_INTAKE_PATH_POWER);
                follower.followPath(pathToIntake2);

                autoState = AutoState.WAIT_FOR_INTAKE2_PATH;
                break;

            case WAIT_FOR_INTAKE2_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                if (!follower.isBusy()) {
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.START_RETURN_PATH2;
                }
                break;

            case START_RETURN_PATH2:
                /*
                 * Flywheel stays off while returning.
                 * This prevents a collected ball from shooting early.
                 */
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot2);

                autoState = AutoState.WAIT_FOR_RETURN_PATH2;
                break;

            case WAIT_FOR_RETURN_PATH2:
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
                    setLauncherSpeedForPose(SHOOT_POSE1);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS2;
                }
                break;

            case SHOOT_INTAKED_BALLS2:
                FlywheelStart();


                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.SHOOT_STOP3;
                }
                break;
            case SHOOT_STOP3:


                if (updateShooter()) {
                    stopFeeders();
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.TURN_TO_ZERO_BEFORE_INTAKE3;
                }
                break;

            case TURN_TO_ZERO_BEFORE_INTAKE3:
                intake.setPower(STOP_SPEED);
                stopFeeders();
                stopFlywheel();

                if (turnToHeadingZeroUsingPedro()) {
                    autoState = AutoState.START_PREINTAKE1_PATH3;
                }
                break;
            case START_PREINTAKE1_PATH3:

                stopFlywheel();

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToPreIntake3);

                autoState = AutoState.START_INTAKE3_PATH;
                break;


            case START_INTAKE3_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                follower.setMaxPower(SLOW_INTAKE_PATH_POWER);
                follower.followPath(pathToIntake3);

                autoState = AutoState.WAIT_FOR_INTAKE3_PATH;
                break;

            case WAIT_FOR_INTAKE3_PATH:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                if (!follower.isBusy()) {
                    intake.setPower(STOP_SPEED);
                    stopFlywheel();

                    autoState = AutoState.START_RETURN_PATH3;
                }
                break;

            case START_RETURN_PATH3:
                /*
                 * Flywheel stays off while returning.
                 * This prevents a collected ball from shooting early.
                 */
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot3);

                autoState = AutoState.WAIT_FOR_RETURN_PATH3;
                break;

            case WAIT_FOR_RETURN_PATH3:
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
                    setLauncherSpeedForPose(SHOOT_POSE2 );

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS3;
                }
                break;

            case SHOOT_INTAKED_BALLS3:
                FlywheelStart();
                intake.setPower(FULL_SPEED);

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(endsequence);
                if (!follower.isBusy()) {
                    stopEverything();
                }
                break;
        }

        showTelemetry();
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
        double blueGoalHeading1 = headingToBlueGoal1(SHOOT_POSE1);
        double blueGoalHeading2 = headingToBlueGoal2(SHOOT_POSE2);


        Pose shootPose1FacingBlue =
                new Pose(SHOOT_POSE1.getX(), SHOOT_POSE1.getY(), blueGoalHeading1);

        Pose shootPose1HeadingZero =
                new Pose(SHOOT_POSE1.getX(), SHOOT_POSE1.getY(), Math.toRadians(0));
        Pose shootPose2FacingBlue =
                new Pose(SHOOT_POSE2.getX(), SHOOT_POSE2.getY(), blueGoalHeading2);

        Pose shootPose2HeadingZero =
                new Pose(SHOOT_POSE2.getX(), SHOOT_POSE2.getY(), Math.toRadians(0));

        pathToFirstShot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, shootPose1FacingBlue))
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(),
                        shootPose1FacingBlue.getHeading()
                )
                .build();

        pathToPreIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1HeadingZero, BEFORE_INTAKE1))
                .setLinearHeadingInterpolation(shootPose1FacingBlue.getHeading() , Math.toRadians(180))
                .build();

        pathToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(BEFORE_INTAKE1, INTAKE_POSE1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pathBackToShot1 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE1, shootPose1FacingBlue))
                .setLinearHeadingInterpolation(
                        INTAKE_POSE1.getHeading(),
                        shootPose1FacingBlue.getHeading()
                )
                .build();

        pathToPreIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1HeadingZero, BEFORE_INTAKE2))
                .setLinearHeadingInterpolation(shootPose1FacingBlue.getHeading() , Math.toRadians(180))
                .build();

        pathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(BEFORE_INTAKE2, INTAKE_POSE2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pathBackToShot2 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE2, shootPose1FacingBlue))
                .setLinearHeadingInterpolation(
                        INTAKE_POSE2.getHeading(),
                        shootPose1FacingBlue.getHeading()
                )
                .build();
        pathToPreIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1HeadingZero, INTAKE_POSE3))
                .setLinearHeadingInterpolation(shootPose1FacingBlue.getHeading(), Math.toRadians(180))
                .build();

        pathToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(BEFORE_INTAKE3, INTAKE_POSE3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pathBackToShot3 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_POSE3, shootPose2FacingBlue))
                .setLinearHeadingInterpolation(
                        INTAKE_POSE3.getHeading(),
                        shootPose2FacingBlue.getHeading()
                )
                .build();
        endsequence = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2FacingBlue, Final_POSE))
                .setLinearHeadingInterpolation(
                        shootPose2FacingBlue.getHeading(),
                        Final_POSE.getHeading()
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


        switch (shooterState) {

            case WAIT_FOR_FLYWHEEL:
                launcher.setVelocity(launcherTargetVelocity);

                /*
                 * CHANGE:
                 * Do not feed a ball until the flywheel has actually ramped up.
                 */
                if (flywheelIsReady()) {
                    intake.setPower(FULL_SPEED);
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
                intake.setPower(STOP_SPEED);
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
        double distance = distanceToBlueGoal(pose);
        launcherTargetVelocity = velocityFromDistance(distance) + kOffset;
        launcher.setVelocity(launcherTargetVelocity);
    }
    private void keepFlywheelReady() {
        setLauncherSpeedForPose(follower.getPose());
    }

    private void FlywheelStart() {
        setLauncherSpeedForPose(follower.getPose());
    }

    private double velocityFromDistance(double x) {
        x = Math.max(18, x);

        return -0.000744119 * x * x * x
                +0.228351 * x * x
                -15.52643 * x
                +1720.40744;
    }

    private double headingToBlueGoal1(Pose pose) {
        double dx = BLUE_GOAL_X - SHOOT_POSE1.getX();
        double dy = BLUE_GOAL_Y - SHOOT_POSE1.getY();

        return Math.atan2(dy, dx) ;
    }
    private double headingToBlueGoal2(Pose pose) {
        double dx = BLUE_GOAL_X - SHOOT_POSE2.getX();
        double dy = BLUE_GOAL_Y - SHOOT_POSE2.getY();

        return Math.atan2(dy, dx) ;
    }


    private double distanceToBlueGoal(Pose pose) {
        return Math.hypot(
                BLUE_GOAL_X - pose.getX(),
                BLUE_GOAL_Y - pose.getY()
        );
    }


    // --------------------------------------------------
    // Telemetry
    // --------------------------------------------------

    private void showTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addData("Auto State", autoState);
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Pose", formatPose(pose));
        telemetry.addData("Distance to Blue", "%.1f", distanceToBlueGoal(pose));
        telemetry.addData("Target Velocity", "%.1f", launcherTargetVelocity);
        telemetry.addData("Actual Velocity", "%.1f", launcher.getVelocity());
        telemetry.addData("Ready to Shoot", flywheelIsReady());
        telemetry.addData("Flywheel Timer", "%.2f", flywheelTimer.seconds());
        telemetry.addData("Blue Heading", "%.1f deg", Math.toDegrees(headingToBlueGoal1(pose)));
        telemetry.addData("Blue Heading 2 ", "%.1f deg", Math.toDegrees(headingToBlueGoal2(pose)));
        telemetry.addData("Heading Error to 0", "%.1f deg", Math.toDegrees(wrapRadians(pose.getHeading())));
        telemetry.addData("Shots", "%d / %d", shotsFinished, shotsWanted);
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
}