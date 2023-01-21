package newArm;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name="Click This one v3", group="Linear Opmode")
public class driveMaster extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    //Motor variables
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftRearDrive;
    private DcMotorEx rightRearDrive;
    private DcMotorEx arm;
    private DcMotorEx claw;
    private Servo clampServo;
    //Encoder TARGET ticks
    private int leftFrontTicks;
    private int rightFrontTicks;
    private int leftRearTicks;
    private int rightRearTicks;


    @Override
    public void runOpMode() {
        initEverything();


        waitForStart();
        while (opModeIsActive()){



            double drive  = gamepad1.left_stick_y * .7;
            double strafe = -gamepad1.left_stick_x * .7;
            double twist  = gamepad1.right_stick_x / 2;
            double[] speeds = {
                    (drive + strafe - twist),
                    (drive - strafe + twist),
                    (drive - strafe - twist),
                    (drive + strafe + twist)
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

            if(gamepad1.dpad_up || gamepad2.dpad_up){
                // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm.setPower(-1);
            }
            else if(gamepad1.dpad_down || gamepad2.dpad_down){
                // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm.setPower(1);
            }
            else{
                arm.setPower(0);

                // arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.right_trigger >= .30) clampServo.setPosition(1);
            else clampServo.setPosition(0);
            if(gamepad2.left_trigger >= .3) claw.setPower(-1);
            if(gamepad2.right_trigger >= .3) claw.setPower(1);
            else claw.setPower(.2);

        }
        telemetry.addData("Arm Ticks", arm.getCurrentPosition());
        telemetry.update();
    }
    public void initEverything () {
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
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(DcMotorEx.class, "claw");
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clampServo = hardwareMap.get(Servo.class, "c");
    }
    /**
     * Initialize the Vuforia localization engine.
     */


}
