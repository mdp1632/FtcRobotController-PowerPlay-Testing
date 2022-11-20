package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {

    DcMotor lift = null;
    double liftSpeed = .5;

    Lift(DcMotor liftMotor){
        DcMotor lift = liftMotor;
    }

    void moveLift(){
        lift.setPower(liftSpeed);
    }


}
