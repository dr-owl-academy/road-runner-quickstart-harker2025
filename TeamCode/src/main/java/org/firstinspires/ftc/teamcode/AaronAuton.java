package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MecanunAaron.MotorTest;

@Autonomous
public class AaronAuton extends OpMode {
    MotorTest motortestAuton = new MotorTest();

    @Override
    public void init() {
        motortestAuton.init(hardwareMap); // Corrected init call
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override // Added Override
    public void start() {

        telemetry.addData("Status", "Running");
        telemetry.update();
        forward(12);
        left(12);


    }

    @Override
    public void loop() {
        //action e.g. forward()
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void forward(double distance) {
        motortestAuton.setMotorSpeed("leftFront", 0.3);
        motortestAuton.setMotorSpeed("rightFront", 0.3);
        motortestAuton.setMotorSpeed("leftBack", 0.3);
        motortestAuton.setMotorSpeed("rightBack", 0.3);

        motortestAuton.rotateInInches(distance, "leftFront");
        motortestAuton.rotateInInches(distance, "rightFront");
        motortestAuton.rotateInInches(distance, "leftBack");
        motortestAuton.rotateInInches(distance, "rightBack");
    }

    public void backward(double distance) {
        motortestAuton.setMotorSpeed("leftFront", -0.3);
        motortestAuton.setMotorSpeed("rightFront", -0.3);
        motortestAuton.setMotorSpeed("leftBack", -0.3);
        motortestAuton.setMotorSpeed("rightBack", -0.3);

        motortestAuton.rotateInInches(distance, "leftFront");
        motortestAuton.rotateInInches(distance, "rightFront");
        motortestAuton.rotateInInches(distance, "leftBack");
        motortestAuton.rotateInInches(distance, "rightBack");
    }

    public void left(double distance) {
        motortestAuton.setMotorSpeed("leftFront", -0.3);
        motortestAuton.setMotorSpeed("rightFront", 0.3);
        motortestAuton.setMotorSpeed("leftBack", 0.3);
        motortestAuton.setMotorSpeed("rightBack", -0.3);

        motortestAuton.rotateInInches(distance, "leftFront");
        motortestAuton.rotateInInches(distance, "rightFront");
        motortestAuton.rotateInInches(distance, "leftBack");
        motortestAuton.rotateInInches(distance, "rightBack");
    }

    public void right(double distance) {
        motortestAuton.setMotorSpeed("leftFront", 0.3);
        motortestAuton.setMotorSpeed("rightFront", -0.3);
        motortestAuton.setMotorSpeed("leftBack", -0.3);
        motortestAuton.setMotorSpeed("rightBack", 0.3);

        motortestAuton.rotateInInches(distance, "leftFront");
        motortestAuton.rotateInInches(distance, "rightFront");
        motortestAuton.rotateInInches(distance, "leftBack");
        motortestAuton.rotateInInches(distance, "rightBack");
    }

    public void CCWTurn(double angle) {
        motortestAuton.setMotorSpeed("leftFront", -0.3);
        motortestAuton.setMotorSpeed("rightFront", 0.3);
        motortestAuton.setMotorSpeed("leftBack", -0.3);
        motortestAuton.setMotorSpeed("rightBack", 0.3);

        motortestAuton.rotateInDegrees(angle, "leftFront");
        motortestAuton.rotateInDegrees(angle, "rightFront");
        motortestAuton.rotateInDegrees(angle, "leftBack");
        motortestAuton.rotateInDegrees(angle, "rightBack");
    }

    public void CWTurn(double angle) {
        motortestAuton.setMotorSpeed("leftFront", 0.3);
        motortestAuton.setMotorSpeed("rightFront", -0.3);
        motortestAuton.setMotorSpeed("leftBack", 0.3);
        motortestAuton.setMotorSpeed("rightBack", -0.3);

        motortestAuton.rotateInDegrees(angle, "leftFront");
        motortestAuton.rotateInDegrees(angle, "rightFront");
        motortestAuton.rotateInDegrees(angle, "leftBack");
        motortestAuton.rotateInDegrees(angle, "rightBack");
    }
}
