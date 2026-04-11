/*
 * Copyright (c) 2024 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* #######################################################
   -------------------------------------------------------
   Copyright (c) Aaron Cheng
   All rights reserved
   Redistribution and use in source and binary forms, with or without modification,
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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

/*
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

@TeleOp(name = "StarterBotTeleOpMecanumAaron", group = "StarterBot")
//@Disabled
public class StarterBotTeleOpMecanumAaron extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    double launcherTargetVelocity = 5000;
    final double LAUNCHER_MIN_VELOCITY = 1750;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

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
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        FEED,
        STOP_FEED
    }


    //The variable that will hold the current state of our launch machine.
    private LaunchState currentLaunchState = LaunchState.IDLE;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotor.class, "intake");


        // Set the drive motor directions and zero power behavior.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        //Set our launcher and feeder motor directions.
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);


        /*
         * Our launcher motor is set to a run mode of "RUN_USING_ENCODER" because we want to set
         * it to a specific velocity instead of a specific power. This allows us to have a more
         * consistent shot.
         * The FTC SDK default PIDF coefficients are tuned for a drive motor, which has a very
         * different dynamic than a launcher. We need to retune them to get a more consistent
         * launcher performance. These values are determined experimentally.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25, 0, 7, 13.5));


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive axis to save power level for telemetry
        double leftStickY = -gamepad1.left_stick_y;  //Remember, this is reversed!
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        //Mecanum drive is controlled with three axes: drive (front-and-back),
        //strafe (side-to-side), and twist (rotating about the center).
        double drive = leftStickY;
        double strafe = leftStickX;
        double twist = rightStickX;

        // You can observe the joystick input by reading the telemetry hint.
        telemetry.addData("Joysticks", "L Y(%.2f), L X(%.2f), R X(%.2f)", leftStickY, leftStickX, rightStickX);

        //Combine the joystick requests for each axis-motion to determine each wheel's power.
        //Set up a variable for each drive wheel to save power level for telemetry.
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = (drive + strafe + twist)/1.5;
        wheelSpeeds[1] = (drive - strafe - twist)/1.5;
        wheelSpeeds[2] = (drive - strafe + twist)/1.5;
        wheelSpeeds[3] = (drive + strafe - twist)/1.5;



        leftFrontDrive.setPower(wheelSpeeds[0]);
        rightFrontDrive.setPower(wheelSpeeds[1]);
        leftBackDrive.setPower(wheelSpeeds[2]);
        rightBackDrive.setPower(wheelSpeeds[3]);



        // Send calculated power to wheels
        telemetry.addData("Wheels", "LF(%.2f), RF(%.2f), LB(%.2f), RB(%.2f)", wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);


        /*
         * This is our state machine for the launcher. The behavior of the GoBilda starter bot
         * is defined by the state of the launcher.
         * The states are IDLE, SPIN_UP, FEED, and STOP_FEED.
         * The default state is IDLE.
         *
         * The triggers for changing state are:
         * 1. A button press on the gamepad.
         * 2. The launcher motor reaching a certain velocity.
         * 3. A timer expiring.
         */


        telemetry.addData("LaunchVelocity", launcherTargetVelocity);

        // Increase target velocity by 100 when dpad_up is pressed
        if (gamepad2.dpad_up && !lastDpadUp) {
            launcherTargetVelocity += 100;
        }
        lastDpadUp = gamepad2.dpad_up;

        // Decrease target velocity by 100 when dpad_down is pressed
        if (gamepad2.dpad_down && !lastDpadDown) {
            launcherTargetVelocity -= 100;
        }
        lastDpadDown = gamepad2.dpad_down;

        switch (currentLaunchState) {
            case IDLE:
                if (gamepad2.leftBumperWasPressed()) {
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0.0);
                }
                if (gamepad2.left_trigger > 0.5) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0.0);
                }
                // If the user presses the "a" button, and we are in the IDLE state,
                // we transition to the SPIN_UP state.
                if (gamepad2.a) {
                    currentLaunchState = LaunchState.SPIN_UP;

                }
                break;

            case SPIN_UP:
                // In this state, we set the launcher motor to our target velocity.
                launcher.setVelocity(launcherTargetVelocity);

                // We can check the current velocity of the motor to see if it has reached
                // our minimum velocity. If it has, we transition to the FEED state.
                if (gamepad2.leftBumperWasPressed()) {
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0.0);
                }
                if (gamepad2.left_trigger > 0.5) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0.0);
                }
                if(launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    currentLaunchState = LaunchState.FEED;
                }

                break;

            case FEED:
                // In this state, we turn on the feeder servos to push the note into the launcher.
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                // We also reset our timer so we can time how long we run the feeder servos.
                feederTimer.reset();
                if (gamepad2.leftBumperWasPressed()) {
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0.0);
                }
                if (gamepad2.left_trigger > 0.5) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0.0);
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
                    if (gamepad2.leftBumperWasPressed()) {
                        intake.setPower(1.0);
                    } else {
                        intake.setPower(0.0);
                    }
                    if (gamepad2.left_trigger > 0.5) {
                        intake.setPower(-1.0);
                    } else {
                        intake.setPower(0.0);
                    }
                    currentLaunchState = LaunchState.IDLE;
                }
                break;
        }


        // Let's add some telemetry data to our driver station, so we can see what's
        // going on with our launcher.
        telemetry.addData("Launcher State", currentLaunchState);
        telemetry.addData("Target Velocity", launcherTargetVelocity);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
