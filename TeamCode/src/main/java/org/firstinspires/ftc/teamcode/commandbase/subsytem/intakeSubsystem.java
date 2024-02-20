package org.firstinspires.ftc.teamcode.commandbase.subsytem;
import static android.os.SystemClock.sleep;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripRotateCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.gripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.outGripperCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.rackCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.sliderCommand;


@Config
public class intakeSubsystem extends SubsystemBase {
    public Servo gripRotate;
    public Servo rack;
    public Servo gripper;
    public DcMotorEx extend;
    //TODO BEAM BREAKER
    public DigitalChannel beamBreaker;

    public static boolean gripped = false;

    public enum ExtensionState{
        INIT,
        EXTEND,RETRACT,
    }
    public enum GripRotateState{
        INIT,
        MID,
        MIDseUPAR,
        MIDseJYAADA,
        DROP,
        PICK,
        AUTO,
        MORE_DROP
    }

    public enum RackState{
        INIT,
        TOPseUPAR,
        TOP,
        MID,
        MIDseJYAADA,
        LOW,
        LOWseUPAR,
        AUTO_TOP_PIXEL,
        AUTO_TOP2,
        AUTO_TOP
    }

    public enum GripperState{
        INIT,
        OPEN,
        CLOSE,
        OPENUP,
        OPENUP_BACK,
        OpenAUTO
    }

   //INITIAL STATE
    GripRotateState gripRotateState = GripRotateState.INIT;
    RackState rackState = RackState.INIT;
    GripperState gripperState = GripperState.INIT;
    ExtensionState extensionState = ExtensionState.INIT;

    public void update(ExtensionState state){
        extensionState = state;
        switch (state){
            case INIT:
                extend.setTargetPosition(extendInitPos);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extend.setPower(extendPower);
                break;
            case EXTEND:
                extend.setTargetPosition(2000);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extend.setPower(extendF);
                //4567
                break;
            case RETRACT:
                extend.setTargetPosition(extend.getCurrentPosition()-100);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extend.setPower(extendF);
                break;
        }
    }

    public void update(GripRotateState state){
        gripRotateState = state;
        switch (state){
            case INIT:
                gripRotate.setPosition(gripRotateInit);
                break;
            case MID:
                gripRotate.setPosition(gripRotateMid);
                break;
            case MIDseUPAR:
                gripRotate.setPosition(gripRotateMidseUPAR);
                break;
            case MIDseJYAADA:
                gripRotate.setPosition(gripRotateMidseJYAADA);
                break;
            case DROP:
                gripRotate.setPosition(gripRotateDrop);
                break;
            case PICK:
                gripRotate.setPosition(gripRotateIntake);
                break;
            case AUTO:
                gripRotate.setPosition(gripRotateAUTO);
                break;
            case MORE_DROP:
                gripRotate.setPosition(gripRotateMoreDrop);
                break;
        }
    }

    public void update(GripperState state){
        gripperState = state;
        switch (state){
            case INIT:
                gripper.setPosition(gripInit);
                break;
            case OPEN:
                gripper.setPosition(gripOpen);
                break;
            case OPENUP:
                gripper.setPosition(gripOpenUP);
                break;
            case OpenAUTO:
                gripper.setPosition(gripOpenAUTO);
                break;
            case OPENUP_BACK:
                gripper.setPosition(gripOpenUP_BACK);
                break;
            case CLOSE:
                gripper.setPosition(gripClose);
                break;
        }
    }

    public void update(RackState state){
        rackState = state;
        switch (state){
            case INIT:
                rack.setPosition(rackInit);
                break;
            case TOPseUPAR:
                rack.setPosition(rackTopseUpar);
                break;
            case TOP:
                rack.setPosition(rackTopIn);
                break;
            case MID:
                rack.setPosition(rackMidIn);
                break;
            case MIDseJYAADA:
                rack.setPosition(rackMidseJYAADA);
                break;
            case LOW:
                rack.setPosition(rackLowIn);
                break;
            case LOWseUPAR:
                rack.setPosition(rackLowseUPARIn);
                break;
            case AUTO_TOP_PIXEL:
                rack.setPosition(rackTOP_PIXEL);
                break;
            case AUTO_TOP2:
                rack.setPosition(rackTOP2);
                break;
            case AUTO_TOP:
                rack.setPosition(rackMorethanTop);
                break;
        }
    }

    public static double extendOff =0;
    public static double extendF =0.5 ;
    public static double extendR =-0.5;
    public static int extendInitPos =0;
    public static int extendMaxPos = 4567;
    public double extendPower =0.6;
    //TODO:GRIP_ROTATE
    public static double gripRotateInit =0.2;
    public static double gripRotateIntake =0.24; //25 original //0.3,0.26
    public static double gripRotateDrop =0.72;//0.75;//0.82;//0.78;//0.8;//0.86;
    public static double gripRotateMoreDrop =0.74;//0.75;//0.82;//0.78;//0.8;//0.86;

    public static double gripRotateMid = 0.55;//0.55;//0.5;
    public static double gripRotateMidseUPAR = 0.6;
    public static double gripRotateMidseJYAADA = 0.62;//0.72;
    public static double gripRotateAUTO =0.26888888;

    //TODO:GRIP
    public static double gripInit =0.64;
    public static double gripOpen =0.67;//0.74; //0.45
    public static double gripOpenAUTO =0.7;
    public static double gripOpenUP= 0.64;//0.4;
    public static double gripOpenUP_BACK = 0.63;//0.44;
    public static double gripClose =0.36;//0.3

    //TODO:RACK
    public static double rackInit =0.88 ;
    public static double rackTopseUpar =0.85;//0.74;    //0.725;//0.72; //0.76;
    public static double rackMorethanTop =0.81;//0.74;    //0.725;//0.72; //0.76;

    public static double rackTopIn =0.83;//0.77;//0.74;//0.86;
    public static double rackMidseJYAADA =0.88;//0.92;
    public static double rackMidIn =0.88;//0.92;
    public static double rackLowIn =0.995;//0.985;//0.94;//0.94;//0.99;//1.0 //99 original

    public static double rackLowseUPARIn = 0.945;//0.92;//0.92;

    //////////////////TODO:AUTO RACK POS/////////////////////////
    public static double rackTOP_PIXEL= 0.91;//0.9
    public static double rackTOP2= 0.92;//0.904;//0.919;///0.922;//0.924good<;//0.928//0.93

    double pixelIntakePos = 0.0;
    double toptwoPos = 0.3;
    //    public static double topPixelPos = 0.5;
    //    public static double l1rackRgrippingpos =0.5;
    //    public static double l1rackLgrippingpos =(1-l1rackRgrippingpos);

    public intakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        rack = hardwareMap.get(Servo.class, "rack");
        gripRotate = hardwareMap.get(Servo.class, "gripR");
        gripper = hardwareMap.get(Servo.class,"gripper");
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        beamBreaker=hardwareMap.get(DigitalChannel.class,"beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);
        extend.setDirection(DcMotorEx.Direction.REVERSE);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //TODO INTAKE SERVO

//    public void intakeExtendInit() {
//        extend.setTargetPosition(-20);
//        extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        extend.setPower(0.6);
//    }
    public void intakeExtendOff() {
        extend.setPower(extendOff);
    }

    public void intakeExtend() {
        extend.setPower(extendF);
    }

    public void intakeAutoStart() {
    }

    //TODO: PIXEL PASS
    public void gripRotateInit() {
        gripRotate.setPosition(gripRotateInit);
    }
    public void gripRotate_pixPick() {
        gripRotate.setPosition(gripRotateIntake);
    }
    public void gripRotate_pixPass() {
        gripRotate.setPosition(gripRotateDrop);
        gripped = false;
    }

    public void gripRotate() {
        double currentPosition = gripRotate.getPosition();
        double newPosition = currentPosition + 0.1;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        rack.setPosition(newPosition);
        sleep(200);
    }


    //TODO: RACK SERVO
    public  void rackInit(){
        rack.setPosition(rackInit);
    }
    public void rackServoINC() {
        double currentPosition = rack.getPosition();
        double newPosition = currentPosition + 0.1;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        rack.setPosition(newPosition);
        sleep(200);

    }
    public void rackServoUP() {
        rack.setPosition(0.68);
//        sleep(200);

    }
    public void rackServoDown() {
        rack.setPosition(0.99);
//        sleep(200);

    }

    public void rackServoDEC() {
        double currentPosition = rack.getPosition();
        double newPosition = currentPosition - 0.1;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        rack.setPosition(newPosition);
        sleep(200);
    }

    public void topPixelPos(){
        rack.setPosition(rackTopIn);
    }

    public void midPixelPos(){
        rack.setPosition(rackMidIn);
    }
    public void lowPixelPos(){
        rack.setPosition(rackLowIn);
    }

    public void takeToptwo(){
        rack.setPosition(rackTopIn);
        gripRotate.setPosition(toptwoPos);
    }


    //TODO: Grip
//    public void gripOpen(){
//        double currentPosition = gripper.getPosition();
//        double newPosition3 = currentPosition + 0.1;
//        newPosition3 = Math.min(1.0, Math.max(0.0, newPosition3));
//        rack.setPosition(newPosition3);
//        sleep(200);
//    }
    public void gripOpen() {
        gripper.setPosition(gripOpen);
    }
    public void gripClose(){
//        if(beamBreaker.getState() == false && !gripped) {
            gripper.setPosition(gripClose);
//            gripped = true;
//        }
    }


//    public void takePixel(){
//        gripper.setPosition(gripInit);
//        sleep(100);
//        gripper.setPosition(gripOpen);
//        sleep(200);
//        gripper.setPosition(gripClose);
//    }
    //TODO: MOTOR EXTENSION
//    public void motor_extension(){
////        extend.setTargetPosition(extendMaxPos);
////        extend.setDirection(DcMotorEx.Direction.REVERSE);
//        extend.setTargetPosition(extend.getCurrentPosition() + 100);
//        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extend.setPower(0.6);
//    }
//    public void motor_retract(){
////        extend.setTargetPosition(extendInitPos);
//        extend.setTargetPosition(extend.getCurrentPosition() - 100);
//        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extend.setPower(0.6);
//    }


    public void take2pixel(){

    }


    }


//RACK
// towards 1 down
//towards 0 up
// MAX EXTEND ENCODER VALUE =
// EXTENSION INIT POS=
// RACK TOP TWO PIXEL HEIGHT (SERVO ANGLE) =
// gripRotate FOR PIXEL INTAKE =
// gripRotate FOR PIXEL TRANSFER =
// RACK HEIGHT (SERVO ANGLE) FOR PIXEL TRANSFER =
// gripperOPEN = 0.6
// gripperCLOSE = 0.35