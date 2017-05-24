package org.firstinspires.ftc.teamcode.FGCOpMode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


enum BallType{
    CONTAMINANT,
    CLEAN,
    UNKNOWN
}

class SorterThread implements Runnable{
    private Thread        thread;
    private DcMotor       motor, shuffleMotor;
    private Servo         leftServo, rightServo;
    private ColDistSensor centerColor, leftColor, rightColor;
    boolean running     = false;

    SorterThread(DcMotor m, DcMotor s, Servo ls, Servo rs, ColDistSensor cc, ColDistSensor cl, ColDistSensor cr){
        thread       = new Thread(this);
        motor        = m;
        shuffleMotor = s;
        leftServo    = ls;
        rightServo   = rs;
        centerColor  = cc;
        leftColor    = cl;
        rightColor   = cr;

        thread.start();
    }

    public void run(){
        while (true) {
            while (this.running) {
                motor.setPower(1);
                shuffleMotor.setPower(1);

                determineServo(leftServo, leftColor);
                determineServo(rightServo, rightColor);
                determineServo(leftServo, rightServo, centerColor);

                sleep(10);
            }

            stopMotors();
            sleep(10);
        }
    }

    void kill(){
        this.running = false;
        thread.interrupt();
    }

    private BallType determineBallType(ColDistSensor sensor){
        float hsv[] = {0, 0, 0};
        Color.colorToHSV(sensor.argb(), hsv);

        float hue = hsv[0];

        if (hue < 30 || hue > 280){
            return BallType.CONTAMINANT;
        } else if (Math.abs(240.0 - hue) < 50.0){
            return BallType.CLEAN;
        }

        return BallType.UNKNOWN;
    }

    private void determineServo(Servo servo, ColDistSensor sensor){
        BallType ball = determineBallType(sensor);

        if(ball != BallType.UNKNOWN && sensor.getDistance(DistanceUnit.MM) < 20){
            switch(ball){
                case CONTAMINANT: servo.setPosition(1.0);
                    break;
                case CLEAN: servo.setPosition(0.0);
                    break;
            }

        } else servo.setPosition(0.5);
    }

    private void determineServo(Servo servo_left, Servo servo_right, ColDistSensor sensor){
        BallType ball = determineBallType(sensor);

        if(ball != BallType.UNKNOWN && sensor.getDistance(DistanceUnit.MM) < 20){
            switch(ball){
                case CONTAMINANT: servo_left.setPosition(1.0);
                    servo_right.setPosition(1.0);
                    break;
                case CLEAN: servo_left.setPosition(0.0);
                    servo_right.setPosition(0.0);
                    break;
            }
        }
    }

    private void sleep(int t){
        try {
            Thread.sleep(t);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void stopMotors(){
        motor.setPower(0);
        shuffleMotor.setPower(0);
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

}
