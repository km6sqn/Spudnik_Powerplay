package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Powerplay Example with Swerve Drive")
@Disabled
public class AiTest extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;
    DcMotor frontLeftSteer;
    DcMotor frontRightSteer;
    DcMotor rearLeftSteer;
    DcMotor rearRightSteer;
    ElapsedTime runtime = new ElapsedTime();

    static final double     TURN_SPEED = 0.5;
    static final double     FORWARD_SPEED = 0.5;
    static final double     DRIVE_CIRCLE = 16;
    static final double     TICKS_PER_DEGREE = DRIVE_CIRCLE / TURN_SPEED;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        frontLeftSteer = hardwareMap.get(DcMotor.class, "front_left_steer");
        frontRightSteer = hardwareMap.get(DcMotor.class, "front_right_steer");
        rearLeftSteer = hardwareMap.get(DcMotor.class, "rear_left_steer");
        rearRightSteer = hardwareMap.get(DcMotor.class, "rear_right_steer");

        // Wait for the start button to be pressed
        waitForStart();
        runtime.reset();

        // Move the robot forward
        moveRobot(FORWARD_SPEED,0);
        // Turn the robot 90 degrees to the right
        //turnRobot(TURN_SPEED, 90);
        // Move the robot back and forth for 2 seconds
        moveRobot(FORWARD_SPEED, -30);
        sleep(1000);
        moveRobot(FORWARD_SPEED, 30);
        sleep(1000);
        moveRobot(FORWARD_SPEED, -30);
        sleep(1000);
        moveRobot(FORWARD_SPEED, 30);
        // Stop the robot
       // stopRobot();
    }

    public void moveRobot(double speed, double distance) {
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
    }
}
