package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="Basic Drive Forward", group="Exercises")
public class MrinaliTurnRight extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFront");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }


    @Override
    public void loop() {

        double forwardSpeed = 0.5;

        leftDrive.setPower(forwardSpeed);
        rightDrive.setPower(forwardSpeed);

        telemetry.addData("Status", "Driving Forward");
        telemetry.update();
    }
}

