package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class SorterThread implements Runnable{
    private Thread       thread;
    private DcMotor     motor;
    private Servo       leftServo, rightServo;
    private ColorSensor centerColor, leftColor, rightColor;
    boolean running = false;

    SorterThread(DcMotor m, Servo ls, Servo rs, ColorSensor cc, ColorSensor cl, ColorSensor cr){
        thread      = new Thread(this);
        motor       = m;
        leftServo   = ls;
        rightServo  = rs;
        centerColor = cc;
        leftColor   = cl;
        rightColor  = cr;

        thread.start();
    }

    public void run(){
        while (true) {
            while (running) {
                motor.setPower(1);

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
        running = false;
        thread.interrupt();
    }

    private void determineServo(Servo servo, ColorSensor sensor){
        if ((float) sensor.red() / sensor.alpha() > 0.4) {
            servo.setPosition(1);
        } else if ((float) sensor.blue() / sensor.alpha() > 0.4) {
            servo.setPosition(0);
        } else {
            servo.setPosition(0.5);
        }
    }

    private void determineServo(Servo servo_left, Servo servo_right, ColorSensor sensor){
        if ((float) sensor.red() / sensor.alpha() > 0.4) {
            servo_left.setPosition(1);
            servo_right.setPosition(1);

        } else if ((float) sensor.blue() / sensor.alpha() > 0.4) {
            servo_left.setPosition(0);
            servo_right.setPosition(0);
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
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }
}
