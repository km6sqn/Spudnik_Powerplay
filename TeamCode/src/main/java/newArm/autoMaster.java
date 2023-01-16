/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package newArm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name="Starting Right With Camera", group="Linear Opmode") //auto mode
public class autoMaster extends LinearOpMode
{

    private final ElapsedTime runtime = new ElapsedTime(); //tim

    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftRearDrive;
    private DcMotorEx rightRearDrive;
    private DcMotorEx arm;
    private DcMotorEx claw;


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

    OpenCvCamera camera;
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

    int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family
    int OtherTag = 2;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode()
    {
        initEverything();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if(tag.id == OtherTag){
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            dropConeInHighTower();
        }
        else if(tagOfInterest.id == ID_TAG_OF_INTEREST){
            dropConeInHighTower();
            strafeSomewhere(changeEncoder(-22));
        }
        else if(tagOfInterest.id == OtherTag){
            dropConeInHighTower();
            strafeSomewhere(changeEncoder(-42));
        }
        else{
            dropConeInHighTower();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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


        claw = hardwareMap.get(DcMotorEx.class, "claw");

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

    //SUMMARIES
    public void dropConeInHighTower(){
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

        goSomewhereCustom(-changeEncoder(5), 250);

        //goSomewhere(-changeEncoder(48));
        arm.setTargetPosition(-2500);
        arm.setVelocity(100);



        while(arm.isBusy()) getTelemetry();
        claw.setPower(-1);
        sleep(1000);
        claw.setPower(0);
        // clamp code
        arm.setTargetPosition(-3000);
        arm.setVelocity(-1000);
        while(arm.isBusy());

        goSomewhere(changeEncoder(3));
        arm.setTargetPosition(0);
        arm.setVelocity(1000);
        strafeSomewhere(changeEncoder(-12));
    }
}