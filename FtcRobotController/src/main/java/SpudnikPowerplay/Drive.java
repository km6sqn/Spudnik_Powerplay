package SpudnikPowerplay;
import android.app.admin.DelegatedAdminReceiver;
import android.widget.Button;
import androidx.annotation.VisibleForTesting;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.Locale;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.Consumer;
//import org.firstinspires.ftc.robotcore.external.hardware.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
@TeleOp(name="SpudnikPowerPlayDriveTest", group="Linear Opmode")
@Disabled
public class Drive extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    //Motor variables
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftRearDrive;
    private DcMotorEx rightRearDrive;
    private DcMotorEx arm;

    //Encoder TARGET ticks
    private int leftFrontTicks;
    private int rightFrontTicks;
    private int leftRearTicks;
    private int rightRearTicks;

    //dPad controls
    private boolean isDpadUp = true;
    private boolean isDpadDown = true;
    private boolean isDpadLeft = true;
    private boolean isDpadRight = true;

    private int mode = 1;
    //0 Run to pos.
    //1 Without encoders;


    public static final String TAG = "Vuforia Navigation Sample";
    OpenGLMatrix lastLocation = null;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    VuforiaLocalizer vuforia;
    WebcamName webcamName;

    @Override
    public void runOpMode(){

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
       // leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDrive.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMotorEnable();


        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightFrontDrive.setTargetPosition(0);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMotorEnable();

        leftRearDrive = hardwareMap.get(DcMotorEx.class, "left_rear");
        //leftRearDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftRearDrive.setTargetPosition(0);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMotorEnable();


        rightRearDrive = hardwareMap.get(DcMotorEx.class, "right_rear");
        rightRearDrive.setTargetPosition(0);
        rightRearDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMotorEnable();


        arm = hardwareMap.get(DcMotorEx.class, "arm");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        resetEncoders();
        waitForStart();
        while (opModeIsActive()){
            getTelemetry();
            getControls();



           // leftFrontDrive.setTargetPosition(leftFrontTicks);

            if(mode == 0) goSomewhere(leftFrontTicks, rightFrontTicks, leftRearTicks, rightRearTicks);
            else{
                /*
                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                 double v1 = r * Math.cos(robotAngle) + rightX;
                 double v2 = r * Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * Math.cos(robotAngle) - rightX;
                /*
                if(gamepad1.left_stick_x == 0){
                    v1 = - v1;
                    v2 = - v2;
                    v3 = - v3;
                    v4 = - v4;
                }


                leftFrontDrive.setPower(v1);
                rightFrontDrive.setPower(v2);
                leftRearDrive.setPower(-v3);
                rightRearDrive.setPower(-v4);
                */

                double drive  = gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double twist  = gamepad1.right_stick_x;
                double[] speeds = {
                        (drive - strafe - twist),
                        (drive + strafe + twist),
                        (drive + strafe - twist),
                        (drive - strafe + twist)
                };
                double max = Math.abs(speeds[0]);
                for(int i = 0; i < speeds.length; i++) {
                    if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
                }

                // If and only if the maximum is outside of the range we want it to be,
                // normalize all the other speeds based on the given speed value.
                if (max > 1) {
                    for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
                }

                // apply the calculated values to the motors.
                leftFrontDrive.setPower(speeds[0]);
                rightFrontDrive.setPower(speeds[1]);
                leftRearDrive.setPower(speeds[2]);
                rightRearDrive.setPower(speeds[3]);

                if(gamepad2.dpad_up) arm.setPower(1);
                else if(gamepad2.dpad_down) arm.setPower(-1);
                else{
                    arm.setPower(0);
              //      arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior);
                }

            }

        }
    }

     public void goSomewhere(int leftFront, int rightFront, int leftRear, int rightRear){
       // resetEncoders();
        leftFrontDrive.setTargetPosition(leftFront);
        rightFrontDrive.setTargetPosition(rightFront);
        leftRearDrive.setTargetPosition(leftRear);
        rightRearDrive.setTargetPosition(rightRear);

        if(leftFront < 0) leftFrontDrive.setVelocity(2000);
        else  leftFrontDrive.setVelocity(-2000);

        if(rightFront < 0) rightFrontDrive.setVelocity(2000);
        else rightFrontDrive.setVelocity(-2000);

        if(leftRear < 0) leftRearDrive.setVelocity(2000);
        else leftRearDrive.setVelocity(-2000);

        if(rightRear < 0) rightRearDrive.setVelocity(-2000);
        else rightRearDrive.setVelocity(2000);

        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy()){
            getTelemetry();
            getControls();
        }

    }


    public void getControls(){
        if(gamepad1.a) resetEncoders();
        if(gamepad1.b) turnEncodersOff();
        if(gamepad1.x) turnPosOn();
        if(gamepad1.y) testMotors();


        if(gamepad1.dpad_up && isDpadUp == true){
            leftFrontTicks += 1000;
            leftRearTicks -= 1000;
            rightFrontTicks += 1000;
            rightRearTicks -= 1000;
            isDpadUp = false;
            sleep(10);
        }
        else if( !isDpadUp && !gamepad1.dpad_up) isDpadUp = true;

        if(gamepad1.dpad_down && isDpadDown == true){
            leftFrontTicks -= 1000;
            leftRearTicks += 1000;
            rightFrontTicks -= 1000;
            rightRearTicks += 1000;
            isDpadDown = false;
            sleep(10);
        }
        else if(!isDpadDown && !gamepad1.dpad_down) isDpadDown = true;

        if(gamepad1.dpad_left && isDpadLeft){
            leftFrontTicks -= 1000;
            leftRearTicks -= 1000;
            rightFrontTicks += 1000;
            rightRearTicks += 1000;
            isDpadLeft = false;
            sleep(10);
        }
        else if(!isDpadLeft && !gamepad1.dpad_left) isDpadLeft = true;

        if(gamepad1.dpad_left && isDpadLeft){
            leftFrontTicks -= 1000;
            leftRearTicks -= 1000;
            rightFrontTicks += 1000;
            rightRearTicks += 1000;
            isDpadLeft = false;
            sleep(10);
        }
        else if(!isDpadLeft && !gamepad1.dpad_left) isDpadLeft = true;

        if(gamepad1.dpad_right && isDpadRight){
            leftFrontTicks += 1000;
            leftRearTicks += 1000;
            rightFrontTicks -= 1000;
            rightRearTicks -= 1000;
            isDpadRight = false;
            sleep(10);
        }
        else if(!isDpadRight && !gamepad1.dpad_right) isDpadRight = true;

    }



    public void turnEncodersOn(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void turnEncodersOff(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        mode = 1;
    }

    public void turnPosOn(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mode = 0;
    }

    public void stopMode(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void testMotors(){
        leftFrontDrive.setPower(1);
        sleep(1000);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(1);
        sleep(1000);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(1);
        sleep(1000);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(1);
        sleep(1000);
        rightRearDrive.setPower(0);
    }

    public void getTelemetry(){
        telemetry.addData("Status", "Run Time: " + runtime.toString()); //run time telemetry
        telemetry.addData("Gamepad Y Stick",-gamepad1.left_stick_y);
        telemetry.addData("GamePad X Stick", gamepad1.left_stick_x);
        telemetry.addData("Gamepad A Button", gamepad1.a);
        telemetry.addData("GamePad X Button", gamepad1.x);
        telemetry.addData("GamePad B Button", gamepad1.b);
        telemetry.addData("isDpadUp", isDpadUp);

        telemetry.addData("", "---------------------------------");

        telemetry.addData("Left Front Drive Target", leftFrontTicks);
        telemetry.addData("Left Front Drive Current", leftFrontDrive.getCurrentPosition());

        telemetry.addData("Right Front Drive Target",  rightFrontTicks);
        telemetry.addData("Right Front Drive Current", rightFrontDrive.getCurrentPosition());

        telemetry.addData("Left Rear Drive Target", leftRearTicks);
        telemetry.addData("Left Read Drive Current", leftRearDrive.getCurrentPosition());

        telemetry.addData("right Rear Drive Target", rightRearTicks);
        telemetry.addData("Right Rear Drive Current", rightRearDrive.getCurrentPosition());

        telemetry.addData("", "---------------------------------");



        telemetry.update();
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



}
