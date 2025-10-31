package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;

@TeleOp
public class Tutorial extends OpMode {

    MotorTest motortest1 = new MotorTest();

    @Override
    public void init(){
        motortest1.init(hardwareMap);
    }

    @Override

    public void loop() {
        double motorSpeed = gamepad1.left_stick_y*-1;

        motortest1.setMotorSpeed(motorSpeed);
        telemetry.addData("motor rev", motortest1.getMotorRevs());
        telemetry.addData("left stick y", gamepad1.left_stick_y);


    }


}
