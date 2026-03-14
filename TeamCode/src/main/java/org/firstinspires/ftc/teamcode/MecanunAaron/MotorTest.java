package org.firstinspires.ftc.teamcode.MecanunAaron;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class MotorTest {
    private HardwareMap hwmap;
    private double ticksPerRev = 537.7;
    //wheel circumference in inches
    private double wheelCircumference = Math.PI * 104 / 25.4;
    private double inPerTick = wheelCircumference / ticksPerRev;


    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;
        // Initializes the motor leftFront.
        DcMotor motor = hwmap.get(DcMotor.class, "leftFront");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializes the motor rightFront.
        motor = hwmap.get(DcMotor.class, "rightFront");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initializes the motor leftBack.
        motor = hwmap.get(DcMotor.class, "leftBack");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializes the motor rightBack.
        motor = hwmap.get(DcMotor.class, "rightBack");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initializes the motor launcher.
        motor = hwmap.get(DcMotor.class, "launcher");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorSpeed(String motorName, double speed) {
        DcMotor motor = this.hwmap.get(DcMotor.class, motorName);
        motor.setPower(speed);
    }

    public void rotateInInches(double distance, String motorName) {
        int distanceInTicks = (int) Math.round(distance / inPerTick);

        DcMotor motor = this.hwmap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(distanceInTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void rotateInDegrees(double degrees, String motorName) {
        int distanceInTicks = (int) Math.round(degrees / 360 * ticksPerRev);

        DcMotor motor = this.hwmap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(distanceInTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}
