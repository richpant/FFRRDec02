package org.firstinspires.ftc.teamcode.drive;

import static com.qualcomm.robotcore.hardware.DcMotor.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedSideAUTOTestAUTO")
public class TestAUTO extends LinearOpMode {
    private DcMotor liftMotor;
    private DcMotor twist;
    private DcMotor carry;
    private Servo bucket;
    RevBlinkinLedDriver coolLights;

    @Override
    public void runOpMode() {
        twist = hardwareMap.get(DcMotor.class, "twist");
        carry = hardwareMap.get(DcMotor.class, "carry");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        bucket = hardwareMap.get(Servo.class, "bucket");
        coolLights = hardwareMap.get(RevBlinkinLedDriver.class, "coolLights");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //liftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startPose = new Pose2d(-23.6, -69.6, Math.PI / 2);
        drive.setPoseEstimate(startPose);
        coolLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
        //liftMotor.getZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(34.2)
                .addTemporalMarker(() -> lift(1700))
                .addTemporalMarker(() -> bucket.setPosition(0.7))
                .waitSeconds(3)
                .addTemporalMarker(() -> bucket.setPosition(-0.5))
                .addTemporalMarker(() -> lift(-1700))
                .strafeRight(79)
                .forward(29)
                .addTemporalMarker(()-> liftMotor.setPower(-0.4))
                .waitSeconds(2)
                .addTemporalMarker(()-> liftMotor.setPower(0))
                .back(20)
                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
    public void lift(int encod)
    {

            carry.setTargetPosition(encod);

            carry.setMode(RunMode.RUN_TO_POSITION);

            carry.setPower(1);
            while(carry.isBusy())
            {
                sleep(50);
            }
    }

    public void duck(int encod)
    {

        liftMotor.setTargetPosition(encod);

        liftMotor.setMode(RunMode.RUN_TO_POSITION);

        liftMotor.setPower(.7);

        }
    }
