package org.firstinspires.ftc.teamcode.FGCOpMode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="UK OpMode V1.2", group="Collyers UK")
public class FirstOpMode extends LinearOpMode {

    public static Telemetry telemetryProxy;
    public static Gamepad gamepad2Proxy;

    private DcMotor       sorting_motor, shuffle_motor, left_motor, right_motor,
            collection_motor1, collection_motor2, lift_motor, front_eject_motor;
    private Servo         left_sort, right_sort, left_eject, right_eject;
    private SorterThread  Sorter;
    private Boolean       harvesterRunning    = false;
    private ElapsedTime   lastHarvesterToggle = new ElapsedTime();
    private ElapsedTime   lastSorterToggle    = new ElapsedTime();
    private LynxI2cColorRangeSensor color_center, color_left, color_right;
    private double dampener = 1d; // valid range 0-1, 1 = no dampening. thomas edit
    private PowerController leftMotorController, rightMotorController;

    private double convertPowerToCurve(double input){
        /*
         * Converts the joystick's linear input into the non-linear output of the motor
         * By using a power curve, it is easier to control the robot's speed.
         * The power curve also handles control dampening // thomas edit
         */
        return Range.clip(0.7*Math.pow(input*dampener, 3.0) + 0.4*input*dampener, -1.0, 1.0); // thomas edit
        //return Range.clip(0.7*Math.pow(input, 3.0) + 0.4*input, -1.0, 1.0); // thomas edit, commented out
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
        gamepad2Proxy = gamepad2;

        sorting_motor      = hardwareMap.dcMotor.get("sort");
        shuffle_motor      = hardwareMap.dcMotor.get("shuffle");
        left_motor         = hardwareMap.dcMotor.get("left_front");
        right_motor        = hardwareMap.dcMotor.get("right_front");
        collection_motor1  = hardwareMap.dcMotor.get("collect1");
        collection_motor2  = hardwareMap.dcMotor.get("collect2");
        lift_motor         = hardwareMap.dcMotor.get("lift");
        front_eject_motor  = hardwareMap.dcMotor.get("front_eject");
        left_sort          = hardwareMap.servo.get("left_sort");
        right_sort         = hardwareMap.servo.get("right_sort");
        left_eject         = hardwareMap.servo.get("left_eject");
        right_eject        = hardwareMap.servo.get("right_eject");
        color_center       = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("color_center");
        color_left         = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("color_left");
        color_right        = (LynxI2cColorRangeSensor) hardwareMap.colorSensor.get("color_right");

        leftMotorController = new PowerController(left_motor,0.1);
        rightMotorController = new PowerController(right_motor,0.1);

        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        front_eject_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorImplEx) front_eject_motor).setTargetPositionTolerance(1);
        telemetry.addData("Move front ejector to 0 position then press Y", "");
        telemetry.addData("Press B to skip testing", "");
        telemetry.update();

        while(!gamepad1.y && !gamepad1.b) {
            sleep(1);
        }

        boolean test = gamepad1.y;

        telemetry.addLine("Configuring ejector");
        telemetry.update();

        front_eject_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        front_eject_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (test) {
            telemetry.addData("Now testing ejector", "");
            telemetry.update();

            front_eject_motor.setPower(1.0);
            front_eject_motor.setTargetPosition(140);

            sleep(2000);

            front_eject_motor.setPower(1.0);
            front_eject_motor.setTargetPosition(0);

            sleep(2000);

            front_eject_motor.setPower(1.0);
            front_eject_motor.setTargetPosition(-140);

            sleep(2000);

            front_eject_motor.setPower(1.0);
            front_eject_motor.setTargetPosition(0);
        }

        gamepad1.setJoystickDeadzone(0.05f);

        waitForStart();

        Sorter = new SorterThread(sorting_motor, shuffle_motor, left_sort, right_sort, color_center, color_left, color_right);

        int direction = 0;

        while(opModeIsActive()) {
            // Control dampening and movement
            if (gamepad1.x) { //thomas edit start
                dampener = 0.7;
            }else{
                dampener = 1;
            } // thomas edit end
            //left_motor.setPower(convertPowerToCurve(gamepad1.left_stick_y));
            //right_motor.setPower(convertPowerToCurve(gamepad1.right_stick_y));
            leftMotorController.setSpeedSoft(convertPowerToCurve(gamepad1.left_stick_y));
            rightMotorController.setSpeedSoft(convertPowerToCurve(gamepad1.right_stick_y));

            leftMotorController.updateSpeed();
            rightMotorController.updateSpeed();

            // Controls sorting system
            if (gamepad1.a || gamepad2.a) toggleSorter();
            if (gamepad1.b || gamepad2.b) toggleHarvester();

            // Controls lifting system
            if (gamepad1.dpad_up || gamepad2.dpad_up) lift_motor.setPower(1.0);
            if (gamepad1.dpad_down || gamepad2.dpad_down) lift_motor.setPower(-1.0);
            if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad2.dpad_up && !gamepad2.dpad_down) lift_motor.setPower(0);

            // Controls ejector paddles
            if (gamepad1.left_bumper) left_eject.setPosition(1);
            else if (gamepad1.left_trigger > 0.1) left_eject.setPosition(0);
            else left_eject.setPosition(0.5);

            if (gamepad2.left_bumper) left_eject.setPosition(1);
            else if (gamepad2.left_trigger > 0.1) left_eject.setPosition(0);
            else left_eject.setPosition(0.5);

            if (gamepad1.right_bumper) right_eject.setPosition(0);
            else if (gamepad1.right_trigger > 0.1) right_eject.setPosition(1);
            else right_eject.setPosition(0.5);

            if (gamepad2.right_bumper) right_eject.setPosition(0);
            else if (gamepad2.right_trigger > 0.1) right_eject.setPosition(1);
            else right_eject.setPosition(0.5);

            if (gamepad1.dpad_right || gamepad2.dpad_right) direction += 5;
            if (gamepad1.dpad_left || gamepad2.dpad_left) direction -= 5;

            if (gamepad2.x) direction = 0;

            front_eject_motor.setTargetPosition(direction);
            front_eject_motor.setPower(1.0);

            telemetry.clear();
            telemetry.addData("Running", "");
            telemetry.update();

            // Sleep to yeald to other applications
            sleep(10);
        }

        Sorter.kill();
    }
}
