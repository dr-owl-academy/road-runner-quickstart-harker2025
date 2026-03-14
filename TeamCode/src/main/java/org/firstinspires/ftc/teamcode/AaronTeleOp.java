package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanunAaron.MotorTest;

@TeleOp
public class AaronTeleOp extends OpMode {

    MotorTest MotorTest1 = new MotorTest();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized by User");
        MotorTest1.init(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        MotorTest1.setMotorSpeed("leftFront", (speed + turn + strafe)/3);
        MotorTest1.setMotorSpeed("rightFront", (speed - turn - strafe)/3);
        MotorTest1.setMotorSpeed("leftBack", (speed + turn - strafe)/3);
        MotorTest1.setMotorSpeed("rightBack", (speed - turn + strafe)/3);

        telemetry.addData("motor rev", speed);
        telemetry.addData("Speed", gamepad1.left_stick_y);
        telemetry.addData("Turn", gamepad1.right_stick_x);
        telemetry.addData("Strafe", gamepad1.left_stick_x);
        telemetry.addData("Status", "Running");
        telemetry.update();


    }
}





