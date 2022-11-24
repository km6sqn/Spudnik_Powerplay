
package org.firstinspires.ftc.teamcode;

import android.app.admin.DelegatedAdminReceiver;
import android.widget.Button;

import androidx.annotation.VisibleForTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@Autonomous(name="Starting Right", group="Linear Opmode") //auto mode
@Disabled
public class AutoEverythingRight extends LinearOpMode { //class config


    private final ElapsedTime runtime = new ElapsedTime(); //tim

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftRearDrive;
    private DcMotorEx rightRearDrive;
    private DcMotorEx arm;
    private Servo clampServo;


    private BNO055IMU imu = null; //imu declaration
    private Orientation angles; //heading degree variable
    private double curHeading; //numerical heading in double form
    private Acceleration gravity; //acceleration
    private double accX; //numerical acceleration x
    private double accY; //numerical acceleration y
    private Acceleration overall; //overall acceleration
    private double overX; //numerical acceleration overall x
    private double overY; //numerical acceleration overall y
    private Position map; //position of robot on map
    private double mapX; //numerical map position x
    private double mapY; //numerical map position y


    private double leftPower= 0; //declare motor power on the left
    private double rightPower = 0; //declare motor power on the right
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family



    @Override
    public void runOpMode() {

        initEverything();

        while (opModeIsActive()) { //while auto is running

            clampServo.setPosition(0);
            sleep(2500);
            arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(-300);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(-1000);
            sleep(100);
            goSomewhere(-100);
            strafeSomewhere(changeEncoder(31));
            goSomewhere(200);
            goSomewhere(-changeEncoder(21));
            arm.setTargetPosition(-3000);

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(-1000);
            while(arm.isBusy()) getTelemetry();

            goSomewhereCustom(-changeEncoder(6), 250);

            goSomewhere(-changeEncoder(48));
            arm.setTargetPosition(-2500);
            arm.setVelocity(100);



            while(arm.isBusy()) getTelemetry();
            clampServo.setPosition(1);
            // clamp code
            arm.setTargetPosition(0);
            arm.setVelocity(1000);
            goSomewhere(changeEncoder(3));
            strafeSomewhere(changeEncoder(-12));
            sleep(3000);
            getTelemetry(); //gets the telemetry
            getNumbers(); // gets the number for hte teltery


            getTelemetry(); //get the telemetry
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            break; //halt the program


        }

    }

    public int changeEncoder(int input){
        return (int)( ((input * 537.7) + .5) / 10);
    }
    public void turnRight(int degrees){
        resetEncoders();

        leftFrontDrive.setMotorEnable();
        rightFrontDrive.setMotorEnable();
        leftRearDrive.setMotorEnable();
        rightRearDrive.setMotorEnable();
        leftFrontDrive.setTargetPosition(10000);
        rightFrontDrive.setTargetPosition(-10000);
        leftRearDrive.setTargetPosition(10000);
        rightRearDrive.setTargetPosition(-10000);
        leftFrontDrive.setVelocity(2000);
        rightFrontDrive.setVelocity(-2000);
        rightRearDrive.setVelocity(2000);
        leftRearDrive.setVelocity(-2000);


        while(leftFrontDrive.isBusy() || rightFrontDrive.isBusy()){
            getTelemetry();
            getNumbers();
            if(curHeading <= degrees - 50) {
                leftFrontDrive.setVelocity(-1000);
                rightFrontDrive.setVelocity(1000);
                leftRearDrive.setVelocity(-1000);
                rightRearDrive.setVelocity(1000);
            }
            if(curHeading <= degrees - 40) {
                leftFrontDrive.setVelocity(-500);
                rightFrontDrive.setVelocity(500);
                leftRearDrive.setVelocity(-500);
                rightRearDrive.setVelocity(500);
            }
            if(curHeading <= degrees - 5){
                leftFrontDrive.setVelocity(-10);
                rightFrontDrive.setVelocity(10);
                leftRearDrive.setVelocity(-10);
                rightRearDrive.setVelocity(10);
            }
            if(curHeading <= degrees){
                break;
            }

        }

    }

    public void turnLeft(int degrees){
        resetEncoders();

        leftFrontDrive.setMotorEnable();
        rightFrontDrive.setMotorEnable();
        leftRearDrive.setMotorEnable();
        rightRearDrive.setMotorEnable();
        leftFrontDrive.setTargetPosition(-10000);
        rightFrontDrive.setTargetPosition(10000);
        leftRearDrive.setTargetPosition(-10000);
        rightRearDrive.setTargetPosition(10000);
        leftFrontDrive.setVelocity(-2000);
        rightFrontDrive.setVelocity(2000);
        rightRearDrive.setVelocity(-2000);
        leftRearDrive.setVelocity(2000);

        while(leftFrontDrive.isBusy() || rightFrontDrive.isBusy()){
            getTelemetry();
            getNumbers();
            if(curHeading >= degrees - 50) {
                leftFrontDrive.setVelocity(1000);
                rightFrontDrive.setVelocity(-1000);
                leftRearDrive.setVelocity(1000);
                rightRearDrive.setVelocity(-1000);
            }
            if(curHeading >= degrees - 40) {
                leftFrontDrive.setVelocity(500);
                rightFrontDrive.setVelocity(-500);
                leftRearDrive.setVelocity(500);
                rightRearDrive.setVelocity(-500);
            }
            if(curHeading >= degrees - 5){
                leftFrontDrive.setVelocity(10);
                rightFrontDrive.setVelocity(-10);
                leftRearDrive.setVelocity(10);
                rightRearDrive.setVelocity(-10);
            }
            if(curHeading >= degrees){
                break;
            }
        }
    }

    public void goSomewhere(int input){
        resetEncoders();
        leftFrontDrive.setTargetPosition(input);
        rightFrontDrive.setTargetPosition(input);
        rightRearDrive.setTargetPosition(input);
        leftRearDrive.setTargetPosition(input);
        if(input < 0) {
            leftFrontDrive.setVelocity(-1000);
            rightFrontDrive.setVelocity(-1000);
            leftRearDrive.setVelocity(-1000);
            rightRearDrive.setVelocity(-1000);
        }
        else{
            leftFrontDrive.setVelocity(1000);
            rightFrontDrive.setVelocity(1000);
            leftRearDrive.setVelocity(1000);
            rightRearDrive.setVelocity(1000);
        }
        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy()){
            getTelemetry();
            getNumbers();
        }

    }

    public void strafeSomewhere(int input) {
        resetEncoders();
        leftFrontDrive.setTargetPosition(input);
        rightFrontDrive.setTargetPosition(-input);
        leftRearDrive.setTargetPosition(-input);
        rightRearDrive.setTargetPosition(input);
        if(input < 0) {
            leftFrontDrive.setVelocity(-1000);
            rightFrontDrive.setVelocity(1000);
            leftRearDrive.setVelocity(1000);
            rightRearDrive.setVelocity(-1000);
        }
        else{
            leftFrontDrive.setVelocity(1000);
            rightFrontDrive.setVelocity(-1000);
            leftRearDrive.setVelocity(-1000);
            rightRearDrive.setVelocity(1000);
        }
        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy()){
            getTelemetry();
            getNumbers();
        }

    }


    public void resetEncoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftRearDrive.setTargetPosition(0);
        rightRearDrive.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void goSomewhereCustom(int input, int speed){
        resetEncoders();
        leftFrontDrive.setTargetPosition(input);
        rightFrontDrive.setTargetPosition(input);
        rightRearDrive.setTargetPosition(input);
        leftRearDrive.setTargetPosition(input);
        if(input < 0) {
            leftFrontDrive.setVelocity(-speed);
            rightFrontDrive.setVelocity(-speed);
            leftRearDrive.setVelocity(-speed);
            rightRearDrive.setVelocity(-speed);
        }
        else{
            leftFrontDrive.setVelocity(speed);
            rightFrontDrive.setVelocity(speed);
            leftRearDrive.setVelocity(speed);
            rightRearDrive.setVelocity(speed);
        }
        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy()){
            getTelemetry();
            getNumbers();
        }
    }
    public void getNumbers(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //reset calibration
        checkOrientation(); //check the orientation
        checkAcceleration(); //check acceleration/home/william/AndroidStudioProjects/6dayschedule/home/william/AndroidStudioProjects/6dayschedule
        checkOverallAcceleration(); //check overall acceleration
        checkNavigation(); //check navigation

    }

    public void initEverything(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front");
        // leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDrive.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMotorEnable();


        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightFrontDrive.setTargetPosition(0);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMotorEnable();

        leftRearDrive = hardwareMap.get(DcMotorEx.class, "left_rear");
        //leftRearDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftRearDrive.setTargetPosition(0);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMotorEnable();


        rightRearDrive = hardwareMap.get(DcMotorEx.class, "right_rear");
        rightRearDrive.setTargetPosition(0);
        rightRearDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMotorEnable();

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setTargetPosition(0);
        arm.setMotorEnable();
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        /*
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setTargetPosition(0);
        arm.setMotorEnable();
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //imu hardware class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //make new parameters
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES; //degree is the unit
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //in meters per second
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true; //logInput
        parameters.loggingTag          = "IMU"; //logs as IMU (see logcat)
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //log acceleration
        imu.initialize(parameters); //initialize all parameters
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); //calibrate the paramete


        clampServo = hardwareMap.get(Servo.class, "c");



        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
    }


    public void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle;
    }
    public void checkAcceleration(){
        gravity = this.imu.getAcceleration();
        accX = gravity.xAccel;
        accY = gravity.yAccel;
    }
    public void checkOverallAcceleration(){
        overall = this.imu.getOverallAcceleration();
        overX = overall.xAccel;
        overY = overall.yAccel;
    }
    public void checkNavigation(){
        map = this.imu.getPosition();
        mapX = map.x;
        mapY = map.y;
        telemetry.addData("X - Y Map", "X (%2f), Y (%2f)", mapX, mapY);
    }

    public void getTelemetry(){
        telemetry.addData("Degrees", "* (%.2f)", curHeading); //degrees telemetry
        telemetry.addData("X - Y Acceleration", "X (%.2f), Y (%2f)", accX, accY); //acceleration telemetry
        telemetry.addData("X - Y Overall", "X (%.2f), Y (%2f)", overX, overY); //overall acceleration telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString()); //run time telemetry
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower); // wheel telemetry
        telemetry.addData("Clamp", "Trigger1 (%.2f), Trigger2 (%.2f)", gamepad1.right_trigger, gamepad2.right_trigger); //servo clamp telemetry
        telemetry.addData("position", leftFrontDrive.getCurrentPosition());
        telemetry.addData("position", rightFrontDrive.getCurrentPosition());
        telemetry.addData("isBusy", leftFrontDrive.isBusy());
        telemetry.addData("arm Ticks ", arm.getCurrentPosition());
        telemetry.update();


    }


}


