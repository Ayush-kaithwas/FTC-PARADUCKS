package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.intake_pickBack2Pos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.out;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.outake_pickBack2Pos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.pick;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.pickFAST;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SlewRateLimiter;
@Disabled
@Config
@TeleOp(name = "FOC_Paraducks")
public class FieldCentric extends CommandOpMode{
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;

    Servo rack;
    private SampleMecanumDrive drive;
    public static double Yvalue = 0.8;
    public HangerAndDrone drone;
    public static double Xvalue = 0.8 ;
    public static double TurnX  = 0.6;

    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    public static double fw_r = 20;
    public static double str_r = 20;
    boolean lastYState = false;
    boolean currentYState = false;
    boolean yToggleState = false; // Initial state

    boolean lastAState = false;
    boolean currentAState = false;
    boolean aToggleState = false; // Initial state
    boolean lastRBState = false;
    boolean currentRBState = false;
    boolean RBToggleState = false; // Initial state
    boolean lastLEFTState = false;
    boolean currentLEFTState = false;
    boolean leftToggleState = false; // Initial state

//    public BHI260IMU imu;


    @Override
    public void initialize() {
//
        rack = hardwareMap.get(Servo.class, "rack");
        drone = new HangerAndDrone(hardwareMap, telemetry);
        intake=new intakeSubsystem(hardwareMap,telemetry);
        outake = new outakeSubsystem(hardwareMap,telemetry);
        endgame = new HangerAndDrone(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        fw = new SlewRateLimiter(fw_r);
        str = new SlewRateLimiter(str_r);

        drone.LockDrone();
        endgame.hanger_INIT();
        outake.slider_INIT();
        schedule(new outArmCommand(outake,outakeSubsystem.OutArmState.INIT));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
        schedule(new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.INIT));
        schedule(new rackCommand(intake, intakeSubsystem.RackState.LOW));
        schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


//        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR));
//        schedule(new rackCommand(intake, intakeSubsystem.RackState.MIDseJYAADA));
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA));
//        schedule(new intake_init(intake,outake));
    }

    @Override
    public void run() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3), Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * Xvalue,
                        input.getY() * Yvalue,
                        -gamepad1.right_stick_x * 0.8));

        drive.update();

//        Pose2d poseEstimate = drive.getPoseEstimate();
//        Vector2d input = new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y, -1, 1), 3),
//                Math.pow(Range.clip(-gamepad1.left_stick_x, -1, 1),3)).rotated(-poseEstimate.getHeading());
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad1.left_stick_y * Yvalue, //Yvalue
//                        -gamepad1.left_stick_x * Xvalue, //Xvalue
//                        -gamepad1.right_stick_x * TurnX //TurnX
//                )
//        );
//        drive.update();
        super.run();

        //TODO CONTROLS
        boolean UP = gamepad1.dpad_up;
        boolean RIGHT = gamepad1.dpad_right;
        boolean DOWN = gamepad1.dpad_down;
        boolean LEFT = gamepad1.dpad_left;
        boolean RB = gamepad1.right_bumper;
        boolean LB = gamepad1.left_bumper;
        boolean LT = gamepad1.left_trigger>0.8;
        boolean RT = gamepad1.right_trigger>0.8;


//        if(intake.beamBreaker.getState() == false ){
////            gripflag = 0;
//            intake.gripper.setPosition(gripOpen);
//        }
//        else if (intake.beamBreaker.getState() == true ) {
//            intake.gripper.setPosition(gripClose);
////            gripflag = 1;
//        }
        // TODO: Intake Pulley Motor


//        if (gamepad1.a) {
//            schedule(new pick(intake));
//        }
//        if(gamepad1.y){
//                schedule(new pickandout(intake,outake));
//        }
// ...0
//        if (currentGamepad1.start && !previousGamepad1.start) {
//            imu.resetYaw();
//        }
        if(UP){
            schedule(new out(outake));
//            new outArmCommand(outake, outakeSubsystem.OutArmState.DROP);
//            new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
        }
        if (DOWN){
            schedule(new pick(intake,outake));
        }
        if(LEFT){
            schedule(new outake_pickBack2Pos(intake,outake));
        }
        if (RIGHT){
//            schedule(new pick2(intake,outake));
            schedule(new intake_pickBack2Pos(intake,outake));
//            drone.
        }
        if (gamepad1.a){
//            intake.gripOpen();
            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOW));
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
        }
        if (gamepad1.y){
            endgame.Hang2Pos();
//            intake.gripOpen();
//            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
//            schedule(new pickFAST(intake,outake));
        }

        if (gamepad1.b) {
//            schedule(new extensionCommand(intake, intakeSubsystem.ExtensionState.EXTEND));
            schedule(new gripperCommand(intake,intakeSubsystem.GripperState.CLOSE));
//            rack.setPosition(1);
        }
        if (gamepad1.x) {
//            schedule(new extensionCommand(intake, intakeSubsystem.ExtensionState.INIT));
            schedule(new gripperCommand(intake,intakeSubsystem.GripperState.OPEN));
//            rack.setPosition(0);
        }
        if(LT){
//            schedule(new pick12(intake,outake));
            schedule(new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.DROP));
            new WaitCommand(400);
//            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
//            new WaitCommand(400);
//            schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND));
//            new WaitCommand(400);
        }
        if(RT){
            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.CLOSE));
        }
        if(RB){
            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
        }
        if(LB){
            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
        }
        if(gamepad1.start){
            schedule(new ParallelCommandGroup(
                    new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                    new rackCommand(intake, intakeSubsystem.RackState.LOW),
                    new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)));
//                new WaitCommand(1500),));
        }
        if(gamepad1.back){
            drone.ShootDrone();
        }

        if (gamepad2.dpad_up){
            endgame.HangerUp();
        }
        if (gamepad2.dpad_down){
            endgame.HangerDown();
        }
//        if (gamepad2.dpad_up){
////            schedule(new extensionCommand(intake,intakeSubsystem.ExtensionState.EXTEND));
//            schedule(new rackCommand(intake, intakeSubsystem.RackState.MID));
//        }
//
//        if (gamepad2.dpad_down){
////            schedule(new extensionCommand(intake,intakeSubsystem.ExtensionState.INIT));
//            schedule(new rackCommand(intake, intakeSubsystem.RackState.TOPseUPAR));
//        }
//
//        if (gamepad2.a){
//            schedule(new sliderCommand(outake, outakeSubsystem.SliderState.INIT));
//        }
        drive.update();

        telemetry.addData("imu",drive.getPoseEstimate());
        telemetry.addData("imu",drive.getExternalHeading());
        telemetry.addData("Rack_servo", intake.rack.getPosition());
        telemetry.addData("GripRotate_servo", intake.gripRotate.getPosition());
        telemetry.addData("Grip_servo", intake.gripper.getPosition());
        telemetry.addData("left front", drive.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("right front", drive.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left back", drive.leftRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("right back", drive.rightRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("sliderR",outake.sliderR.getCurrentPosition());
        telemetry.addData("sliderL",outake.sliderL.getCurrentPosition());
        telemetry.addData("sliderR Current",outake.sliderR.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("sliderL Current",outake.sliderL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("extend",intake.extend.getCurrentPosition());
        telemetry.addData("extend Current",intake.extend.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("bb state",intake.beamBreaker.getState());
        telemetry.addData("HangerMotor pos: ", endgame.HangerMotor.getCurrentPosition());

        telemetry.update();
    }

}

