package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;

public abstract class MrinaliAutonomus extends OpMode {
    MotorTest motortesAuton = new MotorTest();
    @Override
    public void init(){
        motortesAuton.init(hardwareMap);
    }
    public void start(){
        motortesAuton.setMotorSpeed(0.3);
        motortesAuton.rotateInInches(180.8);
        telemetry.addData("motor rev", motortesAuton.getMotorRevs());
    }
    @Override
    public void loop(){

    }
}

