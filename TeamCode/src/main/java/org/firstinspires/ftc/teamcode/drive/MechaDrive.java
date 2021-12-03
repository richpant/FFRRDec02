package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp(name = "MechaDrive")
public class MechaDrive extends OpMode {
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor liftMotor;
    private DcMotor twist;
    private DcMotor carry;
    private Servo bucket;
    public static final double MID_SERVO = 0.5;
    public static final double LIFT_UP_POWER = 0.45;
    public static final double LIFT_DOWN_POWER = -0.45;
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        twist = hardwareMap.get(DcMotor.class, "twist");
        carry = hardwareMap.get(DcMotor.class, "carry");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        bucket = hardwareMap.get(Servo.class, "bucket");
        double position = 0.5;
        int pos = 0;
        int pos2 = 0;

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void loop() {
        drive();
        carousel();
        intake();
        lift();
        box();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    //-----------------------DRIVE------------------
    public void drive() {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

    //-----------------------------------CARRY------------
    private void carousel() {
        if (gamepad2.right_bumper)
            carry.setPower(0.5);
        else if (gamepad2.left_bumper)
            carry.setPower(-0.5);
        else
            carry.setPower(0.0);
    }

    //----------------INTAKE--------------
    public void intake() {

        if (gamepad2.right_trigger >= 0.1) {
            twist.setPower(.6);
        } else if (gamepad2.left_trigger >= 0.1) {
            twist.setPower(-.6);
        } else {
            twist.setPower(0);
        }

    }

    public void lift() {
        if (gamepad2.a)
            liftMotor.setPower(LIFT_UP_POWER);
        else if (gamepad2.y)
            liftMotor.setPower(LIFT_DOWN_POWER);
        else
            liftMotor.setPower(0.0);
    }


    public void box() {
        if (gamepad2.x)
            bucket.setPosition(0.7);
        else if (gamepad2.b)
            bucket.setPosition(-0.5);
    }
}