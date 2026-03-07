package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Adrian_intake {
if (gamepad2.leftbumperwaspressed()) {
    intake.setPower(1);
    }
if (gamepad2.leftbumperwasreleased()) {
    intake.setPower(0);
    }
}