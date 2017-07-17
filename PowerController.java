package org.firstinspires.ftc.teamcode.FGCOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by thomas on 05/07/17.
 */

public class PowerController{
    private DcMotor motor;
    private double currentSpeed;
    private double acceleration;
    public PowerController(DcMotor motorToControl, double acceleration){
        motor = motorToControl;
        this.acceleration = acceleration;
    }

    public void setSpeedSoft(double targetSpeed){
        if(Math.abs(targetSpeed-currentSpeed) < acceleration){
            currentSpeed = targetSpeed;
        }else{
            if (targetSpeed > currentSpeed){
                currentSpeed += acceleration;
            }else {
                currentSpeed -= acceleration;
            }
        }
    }

    public void setSpeedHard(double targetSpeed){
        currentSpeed = targetSpeed;
    }

    public void updateSpeed(){
        motor.setPower(currentSpeed);
    }
}
