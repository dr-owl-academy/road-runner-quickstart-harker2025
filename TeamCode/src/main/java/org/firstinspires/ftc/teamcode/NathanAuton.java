package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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

@Autonomous(name = "NathanAutonNearBlue", group = "StarterBot")
public class NathanAuton extends OpMode {

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

    private static final Pose START_POSE = new Pose(34.10338101430428, 133.12353706111833, Math.toRadians(90));
    private static final Pose END_POSE = new Pose(51.69386657655304, 17.31452071464019, Math.toRadians(130));

    private static final Pose SHOOT_POSE_1 = new Pose(58.88166449934979, 82.43433029908974, Math.toRadians(0));
    private static final Pose SHOOT_POSE_2 = new Pose(54.8242243047646,15.229752933096522, Math.toRadians(130));

    private static final Pose INTAKE_END_1 = new Pose(23.478982300884958, 82.43433029908974, Math.toRadians(180));
    private static final Pose INTAKE_START_2 = new Pose(19.32054616384916,75.99414824447332, Math.toRadians(180));
    private static final Pose INTAKE_END_2 = new Pose(15.072513455143635,75.99414824447332,Math.toRadians(180));
    private static final Pose INTAKE_START_3 = new Pose(42.816827425266084,58.99624757995533,Math.toRadians(180));
    private static final Pose INTAKE_END_3 = new Pose(23.760700025574927,58.99624757995533,Math.toRadians(180));
    private static final Pose INTAKE_START_4 = new Pose(42.007854263517146,34.59819897175125,Math.toRadians(180));
    private static final Pose INTAKE_END_4 = new Pose(23.920676202860857,34.59819897175125,Math.toRadians(180));




    // -----------------------------
    // Shooter constants
    // -----------------------------
    private static final double FULL_SPEED = 1.0;
    private static final double STOP_SPEED = 0.0;

    private static final double FEED_TIME_SECONDS = 0.20;
    private static final double TIME_BETWEEN_SHOTS_SECONDS = 0.35;

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
    private PathChain pathToIntake2;
    private PathChain pathToIntake3;
    private PathChain pathToIntake4;
    private PathChain pathToFinalShot;
    private PathChain pathToFinalPosition;
    private PathChain pathBackToShot;
    private PathChain pathBackToShot2;
    private PathChain pathBackToShot3;
    private PathChain pathEndIntake1;
    private PathChain pathEndIntake2;
    private PathChain pathEndIntake3;
    private PathChain pathEndIntake4;

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
        START_RETURN_PATH2,
        START_RETURN_PATH3,
        START_RETURN_PATH4,
        WAIT_FOR_RETURN_PATH,
        WAIT_FOR_RETURN_PATH2,
        WAIT_FOR_RETURN_PATH3,
        WAIT_FOR_RETURN_PATH4,
        SHOOT_INTAKED_BALLS,
        SHOOT_INTAKED_BALLS2,
        SHOOT_INTAKED_BALLS3,
        SHOOT_INTAKED_BALLS4,
        START_INTAKE_PATH_2,
        END_INTAKE_PATH_2,
        START_INTAKE_PATH_3,
        END_INTAKE_PATH_3,
        START_INTAKE_PATH_4,
        END_INTAKE_PATH_4,
        MOVE_TO_END,
        WHILE_MOVING_END,
        SHOOT_END,
        END_POSITION,
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
                setLauncherSpeedForPose(SHOOT_POSE_1);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToFirstShot);

                autoState = AutoState.WAIT_FOR_FIRST_PATH;
                break;

            case WAIT_FOR_FIRST_PATH:
                keepFlywheelReady();

                if (!follower.isBusy()) {
                    beginShooting(PRELOAD_BALL_COUNT);
                    autoState = AutoState.SHOOT_PRELOADS;
                }
                break;

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

                follower.setMaxPower(SLOW_INTAKE_PATH_POWER);
                follower.followPath(pathEndIntake1);

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
                    setLauncherSpeedForPose(SHOOT_POSE_1);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS;
                }
                break;

            case SHOOT_INTAKED_BALLS:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.START_INTAKE_PATH_2;
                }
                break;
            case START_INTAKE_PATH_2:

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToIntake2);
                autoState = AutoState.END_INTAKE_PATH_2;
                break;
            case END_INTAKE_PATH_2:
                intake.setPower(1);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathEndIntake2);
                autoState = AutoState.START_RETURN_PATH2;
                break;
            case START_INTAKE_PATH_3:

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToIntake3);
                autoState = AutoState.END_INTAKE_PATH_3;
                break;
            case END_INTAKE_PATH_3:
                intake.setPower(1);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathEndIntake3);
                autoState = AutoState.START_RETURN_PATH3;
                break;
            case START_INTAKE_PATH_4:

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToIntake4);
                autoState = AutoState.END_INTAKE_PATH_2;
                break;
            case END_INTAKE_PATH_4:
                intake.setPower(1);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathEndIntake4);
                autoState = AutoState.START_RETURN_PATH4;
                break;
            case START_RETURN_PATH2:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot2);

                autoState = AutoState.WAIT_FOR_RETURN_PATH2;
                break;
            case START_RETURN_PATH3:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot3);

                autoState = AutoState.WAIT_FOR_RETURN_PATH3;
                break;
            case START_RETURN_PATH4:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToFinalShot);

                autoState = AutoState.WAIT_FOR_RETURN_PATH4;
                break;
            case WAIT_FOR_RETURN_PATH2:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                if (!follower.isBusy()) {
                    /*
                     * CHANGE:
                     * Only now, after the robot has reached the shooting pose,
                     * spin up the flywheel for the second shooting sequence.
                     */
                    setLauncherSpeedForPose(SHOOT_POSE_1);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS2;
                }
                break;
            case WAIT_FOR_RETURN_PATH3:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                if (!follower.isBusy()) {
                    /*
                     * CHANGE:
                     * Only now, after the robot has reached the shooting pose,
                     * spin up the flywheel for the second shooting sequence.
                     */
                    setLauncherSpeedForPose(SHOOT_POSE_1);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS3;
                }
                break;
            case WAIT_FOR_RETURN_PATH4:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                if (!follower.isBusy()) {
                    /*
                     * CHANGE:
                     * Only now, after the robot has reached the shooting pose,
                     * spin up the flywheel for the second shooting sequence.
                     */
                    setLauncherSpeedForPose(SHOOT_POSE_1);

                    beginShooting(INTAKED_BALL_COUNT);
                    autoState = AutoState.SHOOT_INTAKED_BALLS4;
                }
                break;
            case SHOOT_INTAKED_BALLS2:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.START_INTAKE_PATH_3;
                }
                break;
            case SHOOT_INTAKED_BALLS3:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.START_INTAKE_PATH_4;
                }
                break;
            case SHOOT_INTAKED_BALLS4:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.END_POSITION;
                }
                break;
            case MOVE_TO_END:
                stopFlywheel();
                intake.setPower(STOP_SPEED);

                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathBackToShot);
                autoState = AutoState.WHILE_MOVING_END;
                break;
            case WHILE_MOVING_END:
                intake.setPower(FULL_SPEED);
                stopFlywheel();

                if (!follower.isBusy()) {
                intake.setPower(STOP_SPEED);
                stopFlywheel();

                autoState = AutoState.SHOOT_END;
            }
                break;
            case SHOOT_END:
                keepFlywheelReady();

                if (updateShooter()) {
                    stopEverything();
                    autoState = AutoState.END_POSITION;
                }
                break;
            case END_POSITION:
                follower.setMaxPower(NORMAL_PATH_POWER);
                follower.followPath(pathToFinalPosition);
                autoState = autoState.DONE;
                break;
            case DONE:
                stopEverything();
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
        double blueGoalHeading = headingToBlueGoal(SHOOT_POSE_1);

        Pose shootPoseFacingBlue =
                new Pose(SHOOT_POSE_1.getX(), SHOOT_POSE_1.getY(), blueGoalHeading);

        Pose shootPoseHeadingZero =
                new Pose(SHOOT_POSE_1.getX(), SHOOT_POSE_1.getY(), Math.toRadians(0));

        pathToFirstShot = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, shootPoseFacingBlue))
                .setLinearHeadingInterpolation(
                        START_POSE.getHeading(),
                        shootPoseFacingBlue.getHeading()
                )
                .build();

        pathBackToShot = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_END_1, shootPoseFacingBlue))
                .setLinearHeadingInterpolation(
                        INTAKE_END_1.getHeading(),
                        shootPoseFacingBlue.getHeading()
                )
                .build();
        pathBackToShot2 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_END_2, shootPoseFacingBlue))
                .setLinearHeadingInterpolation(
                        INTAKE_END_2.getHeading(),
                        shootPoseFacingBlue.getHeading()
                )
                .build();
        pathBackToShot3 = follower.pathBuilder()
                .addPath(new BezierLine(INTAKE_END_3, shootPoseFacingBlue))
                .setLinearHeadingInterpolation(
                        INTAKE_END_3.getHeading(),
                        shootPoseFacingBlue.getHeading()
                )
                .build();
        pathToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_START_2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_START_3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathToIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_START_4))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pathToFinalShot = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, SHOOT_POSE_2))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .setLinearHeadingInterpolation(
                        SHOOT_POSE_2.getHeading(),
                        shootPoseFacingBlue.getHeading()
                )
                .build();
        pathToFinalPosition = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, END_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();
        pathEndIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, SHOOT_POSE_1))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();
        pathEndIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_START_2))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();
        pathEndIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_START_3))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();
        pathEndIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseHeadingZero, INTAKE_START_4))
                .setConstantHeadingInterpolation(Math.toRadians(130))
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
    // Flywheel speed and blue-goal aiming
    // --------------------------------------------------

    private void setLauncherSpeedForPose(Pose pose) {
        double distance = distanceToBlueGoal(pose);
        launcherTargetVelocity = velocityFromDistance(distance) + kOffset;
        launcher.setVelocity(launcherTargetVelocity);
    }

    private void keepFlywheelReady() {
        setLauncherSpeedForPose(follower.getPose());
    }

    private double velocityFromDistance(double x) {
        x = Math.max(18, x);

        return -0.000744119 * x * x * x
                +0.228351 * x * x
                -15.52643 * x
                +1716.40744;
    }

    private double headingToBlueGoal(Pose pose) {
        double dx = BLUE_GOAL_X - pose.getX();
        double dy = BLUE_GOAL_Y - pose.getY();

        return Math.atan2(dy, dx);
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
        telemetry.addData("Blue Heading", "%.1f deg", Math.toDegrees(headingToBlueGoal(pose)));
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