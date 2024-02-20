package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class bot1 extends LinearOpMode {
    String gripper_condition = "";
    /* Declare OpMode members. */
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx rightRear;
    DcMotorEx leftRear;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "LFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "LRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "RRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFront");

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input;//.rotated(-poseEstimate.getHeading());
            input = new Vector2d(-gamepad1.left_stick_y , -gamepad1.left_stick_x );


            drive.setWeightedDrivePower(
                    new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x
                    ));
            drive.update();


//
//                telemetry.addData("ArmL Current (A)", ArmLCurrent);
//                telemetry.addData("ArmR Current (A)", ArmRCurrent);

            telemetry.addData("left front", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right front", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left back", leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right back", rightRear.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("leftF", leftFront.getCurrentPosition());
            telemetry.addData("leftR", leftRear.getCurrentPosition());
            telemetry.addData("rightF", rightFront.getCurrentPosition());
            telemetry.addData("rightR", rightRear.getCurrentPosition());
            // telemetry.addData("hang",hangerMotor.analogInput.getVoltage() );
            //double currentVoltage = getVoltage();
            telemetry.update();
        }
    }
}
