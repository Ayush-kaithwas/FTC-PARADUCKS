package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.out;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.out1;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.outake_pickBack2Pos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.pick;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SlewRateLimiter;
@Disabled
@Config
@TeleOp(name = "FOC2testtttttttttt")
public class FOC2_testttttttttttt extends CommandOpMode {

    intakeSubsystem intake = null;
    outakeSubsystem outake = null;
    HangerAndDrone endgame = null;
    Servo rack;
    private SampleMecanumDrive drive;
    public HangerAndDrone drone;
//    List<LynxModule> allHubs = null;
    public static double Yvalue = 0.9;
    public static double Xvalue = 0.9 ;
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
    boolean lastStartState = false;
    boolean currentStartState = false;
    boolean StartToggleState = false; // Initial state
    boolean lastLEFTState = false;
    boolean currentLEFTState = false;
    boolean leftToggleState = false; // Initial state
    boolean lastUP2State = false;
    boolean currentUP2State = false;
    boolean up2ToggleState = false; // Initial state
    boolean lastDOWN2State = false;
    boolean currentDOWN2State = false;
    boolean down2ToggleState = false; // Initial state
    boolean lastLEFT2State = false;
    boolean currentLEFT2State = false;
    boolean left2ToggleState = false; // Initial state
    double fieldOrientationOffset = 0.0;
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
        endgame.hanger_INIT();
        endgame.HangLock1Init();
        endgame.HangLock2Init();
        schedule(new outArmCommand(outake,outakeSubsystem.OutArmState.INIT));
        schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
        schedule(new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.INIT));
        schedule(new rackCommand(intake, intakeSubsystem.RackState.LOW));
        schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
        schedule(new sliderCommand(outake, outakeSubsystem.SliderState.INIT));


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
        if (gamepad1.left_trigger > 0.7) {
            // Reset the field orientation offset based on the current robot heading
            fieldOrientationOffset = poseEstimate.getHeading();
        }

        // Adjust the heading of the input vector based on the field orientation offset
        double adjustedHeading = poseEstimate.getHeading() - fieldOrientationOffset;
        Vector2d input = new Vector2d(
                Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
                Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)
        ).rotated(-adjustedHeading);

//                str.calculate(drive.y)/2,
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * Xvalue,
                        input.getY() * Yvalue,
                        -gamepad1.right_stick_x * 0.8));

        drive.update();
        super.run();
        //TODO CONTROLS
        currentYState = gamepad1.y;
        currentAState = gamepad1.a;
        currentRBState = gamepad1.right_bumper;
        currentLEFTState = gamepad1.dpad_left;
        currentStartState = gamepad1.start;
        currentUP2State = gamepad2.dpad_up;
        currentDOWN2State = gamepad2.dpad_down;
        currentLEFT2State = gamepad2.dpad_left;
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
//        if (currentLEFTState && !lastLEFTState) {
//            leftToggleState = !leftToggleState;
//            if (leftToggleState) {
//                schedule(new outake_pickBack2Pos(intake,outake));
//            } else {
//                schedule(new intake_pickBack2Pos(intake, outake));
//            }
//        }
                if(LEFT){
            schedule(new outake_pickBack2Pos(intake,outake));
        }
        if (RIGHT){
            schedule(new out1(outake));
//            schedule(new intake_pickBack2Pos(intake,outake));
//            drone.
        }
        if (gamepad1.a){
//            intake.gripOpen();
            schedule(new rackCommand(intake, intakeSubsystem.RackState.LOW));
            schedule(new gripRotateCommand(intake, intakeSubsystem.GripRotateState.MID));
        }
        if (currentYState && !lastYState) {
            yToggleState = !yToggleState;

            if (yToggleState) {
                schedule(new ParallelCommandGroup(
                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                        new rackCommand(intake, intakeSubsystem.RackState.LOWseUPAR),
                        new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)));
            } else {
                schedule(new ParallelCommandGroup(
                        new gripRotateCommand(intake, intakeSubsystem.GripRotateState.PICK),
                        new rackCommand(intake, intakeSubsystem.RackState.LOW),
                        new gripperCommand(intake, intakeSubsystem.GripperState.OPEN)));
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
//        if(LT){
////            schedule(new pick12(intake,outake));
//            schedule(new outArmRotateCommand(outake, outakeSubsystem.OutArmRotateState.DROP));
//            new WaitCommand(400);
////            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.DROP));
////            new WaitCommand(400);
////            schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.EXTEND));
////            new WaitCommand(400);
//        }
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
        if (currentStartState && !lastStartState) {
            StartToggleState = !StartToggleState;

            if (StartToggleState) {
                endgame.HangLock1Open();
                endgame.HangLock2Open();
            }
            else {
                endgame.Hang2Pos();            }
        }

        if(gamepad1.back){
            drone.ShootDrone();
        }

        if (currentUP2State && !lastUP2State) {
            up2ToggleState = !up2ToggleState;

            if (up2ToggleState) {
                schedule(new sliderCommand(outake, outakeSubsystem.SliderState.HIGH));
            }
            else {
                schedule(new sliderCommand(outake, outakeSubsystem.SliderState.RETRACT));
                schedule(new WaitCommand(300));
                schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                schedule(new WaitCommand(300));
                schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));}
        }
//        if (gamepad2.dpad_up){
//            schedule(new sliderCommand(outake, outakeSubsystem.SliderState.MID));
//        }
        if (currentDOWN2State && !lastDOWN2State) {
            down2ToggleState = !down2ToggleState;

            if (down2ToggleState) {
                schedule(new sliderCommand(outake, outakeSubsystem.SliderState.LOW));
            }
            else {
                schedule(new sliderCommand(outake, outakeSubsystem.SliderState.RETRACT));
                schedule(new WaitCommand(300));
                schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                schedule(new WaitCommand(300));
                schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));}
        }
        if (currentLEFT2State && !lastLEFT2State) {
            left2ToggleState = !left2ToggleState;

            if (left2ToggleState) {
                schedule(new sliderCommand(outake, outakeSubsystem.SliderState.MID));
            }
            else {
                schedule(new sliderCommand(outake, outakeSubsystem.SliderState.RETRACT));
                schedule(new WaitCommand(300));
                schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
                schedule(new WaitCommand(300));
                schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));}
        }
        if(gamepad2.right_trigger>0.5){
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.SlightmoreDROP));
            schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.PIXPICK));
        }
        if(gamepad2.left_trigger>0.5){
            schedule(new outArmExtensionCommand(outake, outakeSubsystem.OutArmExtensionState.INIT));
            schedule(new outArmCommand(outake, outakeSubsystem.OutArmState.PICK));
        }
        if (gamepad2.right_bumper){
            double newPosition = outake.outL.getPosition() -0.05;
            double newPosition1 = outake.outR.getPosition() + 0.05;
            newPosition = Math.min(1.0, Math.max(0.0, newPosition));
            newPosition1 = Math.min(1.0, Math.max(0.0, newPosition1));
            outake.outL.setPosition(newPosition);
            outake.outR.setPosition(newPosition1);
            sleep(200);
        }
        if (gamepad2.left_bumper){
            double newPosition = outake.outArm_extend.getPosition() + 0.05;
            newPosition = Math.min(1.0, Math.max(0.0, newPosition));
            outake.outArm_extend.setPosition(newPosition);
            sleep(200);

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
        lastStartState = currentStartState;
        lastUP2State = currentUP2State;
        lastDOWN2State = currentDOWN2State;
        lastLEFT2State = currentLEFT2State;



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

