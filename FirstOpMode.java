package org.firstinspires.ftc.teamcode.FGCOpMode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="UK OpMode V1.2", group="Collyers UK")
public class FirstOpMode extends LinearOpMode {

    public static Telemetry telemetryProxy;

    private DcMotor       sorting_motor, shuffle_motor, left_motor, right_motor, collection_motor1, collection_motor2, lift_motor;
    private Servo         left_sort, right_sort, left_eject, right_eject;
    private SorterThread  Sorter;
    private Boolean       harvesterRunning    = false;
    private ElapsedTime   lastHarvesterToggle = new ElapsedTime();
    private ElapsedTime   lastSorterToggle    = new ElapsedTime();
    private LynxI2cColorRangeSensor color_center, color_left, color_right;

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
        telemetry.setAutoClear(false);
        telemetryProxy = telemetry;

        sorting_motor      = hardwareMap.dcMotor.get("sort");
        shuffle_motor      = hardwareMap.dcMotor.get("shuffle");
        left_motor         = hardwareMap.dcMotor.get("left_front");
        right_motor        = hardwareMap.dcMotor.get("right_front");
        collection_motor1  = hardwareMap.dcMotor.get("collect1");
        collection_motor2  = hardwareMap.dcMotor.get("collect2");
        lift_motor         = hardwareMap.dcMotor.get("lift");
        left_sort          = hardwareMap.servo.get("left_sort");
        right_sort         = hardwareMap.servo.get("right_sort");
        left_eject         = hardwareMap.servo.get("left_eject");
        right_eject        = hardwareMap.servo.get("right_eject");
        color_center       = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("color_center");
        color_left         = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("color_left");
        color_right        = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("color_right");

        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        shuffle_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        gamepad1.setJoystickDeadzone(0.05f);

        waitForStart();

        Sorter = new SorterThread(sorting_motor, shuffle_motor, left_sort, right_sort, color_center, color_left, color_right);

        while(opModeIsActive()) {
            left_motor.setPower(convertPowerToCurve(gamepad1.left_stick_y));
            right_motor.setPower(convertPowerToCurve(gamepad1.right_stick_y));

            if (gamepad1.a) toggleSorter();
            if (gamepad1.b) toggleHarvester();
            if (gamepad1.dpad_up) lift_motor.setPower(1.0);
            if (gamepad1.dpad_down) lift_motor.setPower(-1.0);
            if (!gamepad1.dpad_up && !gamepad1.dpad_down) lift_motor.setPower(0);

            if (gamepad1.left_bumper) left_eject.setPosition(1);
            else if (gamepad1.left_trigger > 0.1) left_eject.setPosition(0);
            else left_eject.setPosition(0.5);

            if (gamepad1.right_bumper) right_eject.setPosition(0);
            else if (gamepad1.right_trigger > 0.1) right_eject.setPosition(1);
            else right_eject.setPosition(0.5);

            sleep(10);
        }

        Sorter.kill();
    }
}
