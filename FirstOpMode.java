package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;

@TeleOp(name="Main OpMode", group="Collyers UK")
public class FirstOpMode extends LinearOpMode {

    private DcMotor      sorting_motor, left_motor, right_motor, collection_motor1, collection_motor2;
    private Servo        left_sort, right_sort;
    private ColorSensor  color_center, color_left, color_right;
    private SorterThread Sorter;
    private Boolean      harvesterRunning    = false;
    private ElapsedTime  lastHarvesterToggle = new ElapsedTime();
    private ElapsedTime  lastSorterToggle    = new ElapsedTime();

    private double convertPowerToCurve(double input){
       return Range.clip(0.7*Math.pow(input, 3.0) + 0.4*input, -1.0, 1.0);
    }

    private void toggleHarvester(){
        if (lastHarvesterToggle.milliseconds() > 1000){

            lastHarvesterToggle.reset();

            harvesterRunning = !harvesterRunning;
            collection_motor1.setPower(harvesterRunning ? 1 : 0);
            collection_motor2.setPower(harvesterRunning ? 1 : 0);
        }
    }

    private void toggleSorter(){
        if (lastSorterToggle.milliseconds() > 1000){

            lastSorterToggle.reset();
            Sorter.running = !Sorter.running;
        }
    }

    public void runOpMode(){

        sorting_motor      = hardwareMap.dcMotor.get("sort");
        left_motor         = hardwareMap.dcMotor.get("left_front");
        right_motor        = hardwareMap.dcMotor.get("right_front");
        collection_motor1  = hardwareMap.dcMotor.get("collect1");
        collection_motor2  = hardwareMap.dcMotor.get("collect2");
        left_sort          = hardwareMap.servo.get("left_sort");
        right_sort         = hardwareMap.servo.get("right_sort");
        color_center       = hardwareMap.colorSensor.get("color_center");
        color_left         = hardwareMap.colorSensor.get("color_left");
        color_right        = hardwareMap.colorSensor.get("color_right");

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        gamepad1.setJoystickDeadzone(0.05f);

        waitForStart();

        Sorter = new SorterThread(sorting_motor, left_sort, right_sort, color_center, color_left, color_right);
        
        while(opModeIsActive()) {
            telemetry.addData("leftStick(vanilla) ", "x:%f y:%f", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("leftStick(scaled)", "x:%f y:%f", convertPowerToCurve(gamepad1.left_stick_x), convertPowerToCurve(gamepad1.left_stick_y));
            telemetry.update();

            left_motor.setPower(convertPowerToCurve(gamepad1.left_stick_y - gamepad1.left_stick_x));
            right_motor.setPower(convertPowerToCurve(gamepad1.left_stick_y + gamepad1.left_stick_x));

            if (gamepad1.a) toggleSorter();
            if (gamepad1.b) toggleHarvester();

            sleep(10);
        }

        Sorter.kill();
    }
}
