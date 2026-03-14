package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;

@Autonomous

public class autonTutorial extends OpMode{
    MotorTest motortestAuton = new MotorTest();

    @Override
    public void init(){
        motortestAuton.init(hardwareMap);
    }

    public void start(){
        motortestAuton.setMotorSpeed(0.1);
        motortestAuton.rotateInInches(12.8);
        telemetry.addData("motor rev", motortestAuton.getMotorRevs());

    }

    @Override
    public void loop(){

    }
}