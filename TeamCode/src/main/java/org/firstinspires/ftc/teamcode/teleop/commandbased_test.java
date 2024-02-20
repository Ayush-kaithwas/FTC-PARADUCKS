package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem.gripClose;
import static org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem.gripOpen;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.extensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmExtensionCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outArmRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.sliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.subsytem.outakeSubsystem;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.intake_init;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.intake_pickBack2Pos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.out;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.pick;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.outake_pickBack2Pos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.pickFAST;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SlewRateLimiter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
@Disabled
@Config
@TeleOp(name = "CommandBasetest")
public class commandbased_test extends CommandOpMode {
    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    private SampleMecanumDrive drive;
    public HangerAndDrone drone;
    List<LynxModule> allHubs = null;
    public static double Yvalue = 0.8;
    public static double Xvalue = 0.8 ;
    public static double TurnX  = 0.6;
    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    public static double fw_r = 20;
    public static double str_r = 20;
    int gripflag =0; //Gripper close
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
    @Override
    public void initialize() {
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
        outake.slider_INIT();
        schedule(new outArmCommand(outake,outakeSubsystem.OutArmState.INIT));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
        schedule(new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.INIT));
        schedule(new rackCommand(intake, intakeSubsystem.RackState.LOW));
        schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));


//        schedule(new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE));
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseUPAR));
//        schedule(new rackCommand(intake, intakeSubsystem.RackState.MIDseJYAADA));
//        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MIDseJYAADA));
//        schedule(new intake_init(intake,outake));
    }

    @Override
    public void run() {



//            if (currentAState && !lastAState) {
//                // Toggle condition: A button is pressed and was not pressed in the previous loop
//                aToggleState = !aToggleState;
//
//                // Perform actions based on the toggle state for A button
//                if (aToggleState) {
//                    // Schedule commands for the first toggle state (e.g., your first set of commands for A button)
//                    // Add your desired commands for the first toggle state with button A here
//                } else {
//                    // Schedule commands for the second toggle state (e.g., your second set of commands for A button)
//                    // Add your desired commands for the second toggle state with button A here
//                }
//            }

        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(Math.pow(Range.clip(-gamepad1.left_stick_y, -1, 1), 3),
                Math.pow(Range.clip(-gamepad1.left_stick_x, -1, 1),3)).rotated(-poseEstimate.getHeading());
        double fwInput = fw.calculate(input.getX()) * Xvalue;
        double strInput = str.calculate(input.getY()) * Yvalue;
        double turnInput = -gamepad1.right_stick_x * 0.8;
        drive.setWeightedDrivePower(new Pose2d(fwInput, strInput, turnInput));

//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad1.left_stick_y * Yvalue, //Yvalue
//                        -gamepad1.left_stick_x * Xvalue, //Xvalue
//                        -gamepad1.right_stick_x * TurnX //TurnX
//                )
//        );
        drive.update();
        super.run();
        //TODO CONTROLS
        currentYState = gamepad1.y;
        currentAState = gamepad1.a;
        currentRBState = gamepad1.right_bumper;
        currentLEFTState = gamepad1.dpad_left;
        boolean UP = gamepad1.dpad_up;
        boolean RIGHT = gamepad1.dpad_right;
        boolean DOWN = gamepad1.dpad_down;
        boolean LEFT = gamepad1.dpad_left;
        boolean RB = gamepad1.right_bumper;
        boolean LB = gamepad1.left_bumper;
        boolean LT = gamepad1.left_trigger>0.8;
        boolean RT = gamepad1.right_trigger>0.8;

        boolean objectDetected = !intake.beamBreaker.getState();
//        if(objectDetected){
//            intake.gripper.setPosition(gripOpen);
//            gripflag = 0;
//
//        }
//        else if (!objectDetected && gripflag == 0) {
//            intake.gripper.setPosition(gripClose);
//            gripflag = 1;
//        }
        // TODO: Intake Pulley Motor

//        }
        if(UP){
            schedule(new out(outake));
//            new outArmCommand(outake, outakeSubsystem.OutArmState.DROP);
//            new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND);
        }
        if (DOWN){
            schedule(new pick(intake,outake));
        }

        if (currentLEFTState && !lastLEFTState) {
                leftToggleState = !leftToggleState;
                if (leftToggleState) {
                    schedule(new outake_pickBack2Pos(intake,outake));
                } else {
                    schedule(new intake_pickBack2Pos(intake, outake));
                }
            }
        //        if(LEFT){
//            schedule(new outake_pickBack2Pos(intake,outake));
//        }
//        if (RIGHT){
////            schedule(new pick2(intake,outake));
//            schedule(new intake_pickBack2Pos(intake,outake));
////            drone.
//        }
        if (gamepad1.a){
//            intake.gripOpen();
            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOW));
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
        }
        if (currentYState && !lastYState) {
            yToggleState = !yToggleState;

            if (yToggleState) {
                // Schedule commands for the first toggle state (e.g., opening the gripper for Y button)
                schedule(new ParallelCommandGroup(
                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                        new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                        new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)));
            } else {
                // Schedule commands for the second toggle state (e.g., closing the gripper for Y button)
                schedule(new ParallelCommandGroup(
                        new rackCommand(intake, intakeSubsystem.RackState.LOW),
                        new gripperCommand(intake, intakeSubsystem.GripperState.CLOSE)));
            }
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
//        if(RB){
//            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
//        }
        if (currentRBState && !lastRBState) {
            RBToggleState = !RBToggleState;

            if (RBToggleState) {
                schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENRIGHT));
            } else {
                schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
            }
        }
        if(LB){
            schedule(new outGripperCommand(outake, outakeSubsystem.OutGripperState.OPENLEFT));
        }
        if(gamepad1.start){
//            endgame.Hang2Pos();
//            schedule(new ParallelCommandGroup(
//                    new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
//                    new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
//                    new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)));
////                new WaitCommand(1500),));
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
        if (gamepad2.dpad_left){
            endgame.Hang2Pos();
        }
        if (gamepad2.dpad_right){

        }
        //        if (gamepad1.y){
////            intake.gripOpen();
////            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
////            endgame.Hang2Pos();
//            schedule(new pickFAST(intake,outake));
//        }
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
        lastYState = currentYState;
        lastAState = currentAState;
        lastRBState = currentRBState;
        lastLEFTState = currentLEFTState;

        //TODO: TELEMETRY

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
        telemetry.addData("bb state",intake.beamBreaker.getState());
        telemetry.addData("HangerMotor pos: ", endgame.HangerMotor.getCurrentPosition());
        telemetry.addData("HangerMotor Current: ", endgame.HangerMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

//        for (LynxModule hub : allHubs) {
//            hub.clearBulkCache();
//        }
    }


}
