package newBot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.hardware.Consumer;
//import org.firstinspires.ftc.robotcore.external.hardware.Continuation;

@Disabled
@TeleOp(name="Click This one v3", group="Linear Opmode")
public class driveMaster extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    //Motor variables
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftRearDrive;
    private DcMotorEx rightRearDrive;
    private DcMotorEx arm1, arm2;
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
        while (opModeIsActive()) {


            double drive = gamepad1.left_stick_y * .85;
            double strafe = -gamepad1.left_stick_x * .85;
            double twist = gamepad1.right_stick_x * .6;
            double[] speeds = {
                    (drive + strafe - twist),
                    (drive - strafe + twist),
                    (drive - strafe - twist),
                    (drive + strafe + twist)
            };
            double max = Math.abs(speeds[0]);
            for (int i = 0; i < speeds.length; i++) {
                if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
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

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm1.setPower(-1);
                arm2.setPower(-1);

            }
            else if(gamepad1.dpad_down || gamepad2.dpad_down){
                // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                arm1.setPower(1);
                arm2.setPower(1);
            }
            else{
                arm1.setPower(0);
                arm2.setPower(0);

                // arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.a){

                    leftFrontDrive.setPower(1);
                    rightFrontDrive.setPower(1);
                    leftRearDrive.setPower(1);
                    rightRearDrive.setPower(1);

            }

            if(gamepad2.right_trigger >= .30) clampServo.setPosition(1);
            else clampServo.setPosition(0);

            if(gamepad2.left_trigger >= .3) claw.setPower(-1);
            else if(gamepad2.right_trigger >= .3) claw.setPower(.85);
            else claw.setPower(.1);

        }
        telemetry.addData("Arm Ticks", arm1.getCurrentPosition());
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

        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        claw = hardwareMap.get(DcMotorEx.class, "claw");
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clampServo = hardwareMap.get(Servo.class, "c");
    }
    /**
     * Initialize the Vuforia localization engine.
     */


}
