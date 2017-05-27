package org.firstinspires.ftc.teamcode.FGCOpMode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
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
    private LynxI2cColorRangeSensor centerColor, leftColor, rightColor;
    boolean running = false;
    boolean killed = false;

    SorterThread(DcMotor m, DcMotor s, Servo ls, Servo rs, LynxI2cColorRangeSensor cc, LynxI2cColorRangeSensor cl, LynxI2cColorRangeSensor cr){
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
        while (!killed) {
            while (running) {
                shuffleMotor.setPower(1);

                determineServo(leftServo, leftColor, 1.0);
                determineServo(rightServo, rightColor, 0.0);
                determineServo(leftServo, rightServo, centerColor);

                if (leftColor.getDistance(DistanceUnit.MM) < 56 || rightColor.getDistance(DistanceUnit.MM) < 56){
                    motor.setPower(0);
                } else {
                    motor.setPower(1);
                }

                sleep(10);
            }

            stopMotors();
            sleep(10);
        }
    }

    void kill(){
        running = false;
        killed = true;

        stopMotors();
        FirstOpMode.telemetryProxy.addLine("Sorter thread has been killed!");
        FirstOpMode.telemetryProxy.update();
    }

    private BallType determineBallType(LynxI2cColorRangeSensor sensor){
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

    private void determineServo(Servo servo, LynxI2cColorRangeSensor sensor, double default_direction){
        BallType ball = determineBallType(sensor);

        if(sensor.getDistance(DistanceUnit.MM) < 100){
            switch(ball){
                case CONTAMINANT: servo.setPosition(1.0);
                    break;
                case CLEAN: servo.setPosition(0.0);
                    break;
                case UNKNOWN: servo.setPosition(default_direction);
            }

        } else servo.setPosition(0.5);
    }

    private void determineServo(Servo servo_left, Servo servo_right, LynxI2cColorRangeSensor sensor){
        BallType ball = determineBallType(sensor);

        if(ball != BallType.UNKNOWN && sensor.getDistance(DistanceUnit.MM) < 100){
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
            FirstOpMode.telemetryProxy.addData("Interrupted Exception occured", "Line %d", e.getStackTrace()[0].getLineNumber());
            FirstOpMode.telemetryProxy.update();
        }
    }

    private void stopMotors(){
        motor.setPower(0);
        shuffleMotor.setPower(0);
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

}
