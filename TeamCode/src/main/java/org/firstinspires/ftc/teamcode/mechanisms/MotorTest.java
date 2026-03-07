package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class MotorTest {

    private DcMotor motor;
    private double ticksPerRev = 537.7;
    // wheel circumference in inches
    private double wheelCircumference = 104 * Math.PI / 25.4;
    private double inPerTick = wheelCircumference / ticksPerRev;


    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "leftFront");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }

    public double getMotorRevs() {
        return motor.getCurrentPosition()/ticksPerRev;
    }

    public void rotateInInches(double distance) {
        int distanceInTicks = (int) Math.round(distance / inPerTick);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(distanceInTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}

