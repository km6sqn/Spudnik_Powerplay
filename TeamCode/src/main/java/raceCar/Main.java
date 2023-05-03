package raceCar;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class Main extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo turn;
    private Servo spoiler;
    private double servoPlace = 0;

    @Override
    public void init() {
        // Initialize the motors using the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        turn = hardwareMap.get(Servo.class, "t");
        spoiler = hardwareMap.get(Servo.class, "s");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        servoPlace = .5;

    }

    @Override
    public void loop() {
        // Gamepad controls
        double drive = gamepad1.left_trigger + -gamepad1.right_trigger;
        double turnPower  =  1 + gamepad1.right_stick_x;

        // Calculate motor power
        double leftPower  = drive;
        double rightPower = drive;


        // Limit motor power to range [-1, 1]
        leftPower  = Math.max(-1, Math.min(leftPower, 1));
        rightPower = Math.max(-1, Math.min(rightPower, 1));

        // Set motor power
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        turn.setPosition((servoPlace - (turnPower / 10)));

        spoiler.setPosition(-drive / 5);
    }
}
