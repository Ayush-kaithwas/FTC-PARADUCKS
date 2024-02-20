package org.firstinspires.ftc.teamcode.commandbase.subsytem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static android.os.SystemClock.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class outakeSubsystem extends SubsystemBase {

    //VERTICAL MOVEMENT
    public Servo outL;
    public Servo outR;
    public Servo outArm_rotate; //L->R and R->L
    public Servo outArm_extend;
    public Servo outGripperL;
    public Servo outGripperR;

    public DcMotorEx sliderL;
    public DcMotorEx sliderR;

    public enum OutArmExtensionState {
        INIT,
        EXTEND,
        PIXPICK,
        MidEXTEND
    }

    public enum SliderState {
        INIT,
        LOW,
        MID,
        HIGH,
        RETRACT,
    }

    public enum OutArmRotateState {
        INIT,
        DROP,
        PICK
    }

    public enum OutGripperState {
        INIT,
        OPEN,
        CLOSE,
        OPENLEFT,
        OPENRIGHT,
    }

    public enum OutArmState {

        //outL ,outR
        INIT,
        PICK,
        PICKseUPAR,
        MidDROP,
        LessthanDROP,
        DROP,
        DROPseUPAR,
        SlightmoreDROP,
        BFDROPCENTREAUTO, ///bluefield

    }

    public OutArmExtensionState outArmExtensionState = OutArmExtensionState.INIT;
    public static OutArmRotateState outArmRotateState = OutArmRotateState.INIT;
    public OutGripperState outGripperState = OutGripperState.INIT;
    public SliderState sliderState = SliderState.INIT;
    public OutArmState outArmState = OutArmState.INIT;

    public void update(OutArmState state) {
        outArmState = state;
        switch (state) {
            case INIT:
                outL.setPosition(outL_pixInit);
                outR.setPosition(outR_pixInit);
                break;
            case PICK:
                outL.setPosition(outL_pixPick);
                outR.setPosition(outR_pixPick);
                break;
            case PICKseUPAR:
                outL.setPosition(outL_pixPickseUpar);
                outR.setPosition(outR_pixPickseUpar);
                break;
            case MidDROP:
                outL.setPosition(outL_pixMidDrop);
                outR.setPosition(outR_pixMidDrop);
                break;
            case LessthanDROP:
                outL.setPosition(outL_pixLessthanDrop);
                outR.setPosition(outR_pixLessthanDrop);
                break;
            case DROP:
                outL.setPosition(outL_pixDrop);
                outR.setPosition(outR_pixDrop);
                break;
            case DROPseUPAR:
                outL.setPosition(outL_pixDropseUpar);
                outR.setPosition(outR_pixDropseUpar);
                break;
            case BFDROPCENTREAUTO:
                outL.setPosition(outL_pixDROPBLUEFar);
                outR.setPosition(outR_pixDROPBLUEFar);
                break;
            case SlightmoreDROP:
                outL.setPosition(outL_pixSlightmoreDrop);
                outR.setPosition(outR_pixSlightmoreDrop);
        }
    }

    public void update(OutArmExtensionState state) {
        outArmExtensionState = state;
        switch (state) {
            case INIT:
                outArm_extend.setPosition(outArmExtend_pixInit);
                break;
            case PIXPICK:
                outArm_extend.setPosition(outArmExtend_pixPick);
                break;
            case EXTEND:
                outArm_extend.setPosition(outArmExtend_pixDrop);
                //ADD when slider state mis/high then extend
                break;
            case MidEXTEND:
                outArm_extend.setPosition(outArmExtend_Mid);
                //ADD when slider state mis/high then extend
                break;
        }
    }

    public void update(SliderState state) {
        sliderState = state;
        switch (state) {
            case INIT:
                sliderL.setTargetPosition(sliderLInitPos);
                sliderR.setTargetPosition(sliderRInitPos);
                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderL.setPower(0);
                sliderR.setPower(0);
                break;
            case LOW:
                sliderL.setTargetPosition(sliderLlow);
                sliderR.setTargetPosition(sliderRlow);
                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderL.setPower(0.5);
                sliderR.setPower(0.5);
                break;
            case MID:
                sliderL.setTargetPosition(sliderLmid);
                sliderR.setTargetPosition(sliderRmid);
                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderL.setPower(0.5);
                sliderR.setPower(0.5);
                break;
            case HIGH:
                sliderL.setTargetPosition(sliderLhigh);
                sliderR.setTargetPosition(sliderRhigh);
                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderL.setPower(0.5);
                sliderR.setPower(0.5);
                break;
            case RETRACT:
                sliderL.setTargetPosition(sliderLretract);
                sliderR.setTargetPosition(sliderRretract);
                sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderL.setPower(0.5);
                sliderR.setPower(0.5);
                break;
        }
    }

    public void update(OutArmRotateState state) {
        outArmRotateState = state;
        switch (state) {
            case INIT:
                outArm_rotate.setPosition(outArmRotate_pixInit);
                break;
            case DROP:
                outArm_rotate.setPosition(outArmRotate_pixDrop);
                break;
            case PICK:
                outArm_rotate.setPosition(outArmRotate_pixPick);
                break;
        }
    }

    public void update(OutGripperState state) {
        outGripperState = state;
        switch (state) {
            case INIT:
                outGripperL.setPosition(outGripperL_pixInit);
                outGripperR.setPosition(outGripperR_pixInit);
                break;
            case OPEN:
                outGripperL.setPosition(outGripperL_pixDrop);
                outGripperR.setPosition(outGripperR_pixDrop);
                break;
            case CLOSE:
                outGripperL.setPosition(outGripperL_pixPick);
                outGripperR.setPosition(outGripperR_pixPick);
                break;
            case OPENLEFT:
                outGripperL.setPosition(outGripperL_pixDrop);
                break;
            case OPENRIGHT:
                outGripperR.setPosition(outGripperR_pixDrop);
                break;
        }
    }

    //TODO: Pixel Transfer

    public static double outL_pixPick = 0.22;
    public static double outR_pixPick = (1 - outL_pixPick);//0.75;
    public static double outL_pixInit = 0.21;
    public static double outR_pixInit = (1 - outL_pixInit);
    public static double outGripperL_pixPick = 0.825;//0.815;//0.82;
    public static double outGripperR_pixPick = 0.178;//0.18;
    public static double outGripperL_pixInit = 0.0;
    public static double outGripperR_pixInit = 0.0;

    public static double outArmExtend_pixInit = 0.2;
    public static double outArmExtend_pixPick = 0.35;
    public static double outArmRotate_pixPick = 0.735;
    public static double outL_pixDrop = 0.645;//0.615;//0.395;
    public static double outR_pixDrop = (1 - outL_pixDrop);//0.385;//0.595;
    public static double outL_pixSlightmoreDrop = 0.675;//0.615;//0.395;
    public static double outR_pixSlightmoreDrop = (1 - outL_pixSlightmoreDrop);//0.385;//0.595;
    public static double outL_pixLessthanDrop = 0.6;//0.615;//0.395;
    public static double outR_pixLessthanDrop = (1 - outL_pixLessthanDrop);
    public static double outL_pixMidDrop = 0.45;//0.615;//0.395;
    public static double outR_pixMidDrop = (1 - outL_pixLessthanDrop);
    public static double outL_pixDropseUpar = 0.72;//0.715;
    public static double outR_pixDropseUpar = (1 - outL_pixDropseUpar);
    public static double outL_pixDROPBLUEFar = 0.695;//0.715;
    public static double outR_pixDROPBLUEFar = (1 - outL_pixDROPBLUEFar);
    public static double outL_pixPickseUpar = 0.25;
    public static double outR_pixPickseUpar = (1 - outL_pixPickseUpar);
    public static double outArmRotate_pixDrop = 0.75;
    public static double outArmExtend_pixDrop = 0.8;//0.65;
    public static double outArmExtend_Mid = 0.55;//0.65;

    public static double outGripperL_pixDrop = 0.54;
    public static double outGripperR_pixDrop = (1 - outGripperL_pixDrop);// 0.35;
    public static double outArmRotate_pixInit = 0.73;
    public static double servoInc = 0.1;
    public static int sliderLlow = 320;
    public static int sliderRlow = 320;
    public static int sliderLmid = 735;
    public static int sliderRmid = 735;
    public static int sliderLhigh = 1316;
    public static int sliderRhigh = 1316;
    public static int sliderLInitPos = 0;
    public static int sliderRInitPos = 0;

    public static int sliderLretract = 0;
    public static int sliderRretract = 0;


    public outakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        outL = hardwareMap.get(Servo.class, "outL"); //port1
        outR = hardwareMap.get(Servo.class, "outR");//port0
        outArm_rotate = hardwareMap.get(Servo.class, "outRotate");//port2
        outArm_extend = hardwareMap.get(Servo.class, "outEx"); //outEx//port5
        outGripperL = hardwareMap.get(Servo.class, "outGripperL");//port4
        outGripperR = hardwareMap.get(Servo.class, "outGripperR");//port3
        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
// TODO: OUTARM GRIP

    public void outgrip() {
        outGripperL.setPosition(outGripperL_pixPick);
        outGripperL.setPosition(outGripperR_pixPick);
    }

    public void outdrop() {
        outGripperL.setPosition(outGripperL_pixDrop);
        outGripperL.setPosition(outGripperR_pixDrop);
    }

    //TODO: SLIDER EXTEND
    public void sliderR_extend() {
//        extend.setTargetPosition(extendMaxPos);
        sliderR.setTargetPosition(sliderR.getCurrentPosition() + 100);
        sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderR.setPower(0.6);
    }

    public void sliderR_retract() {
//        extend.setTargetPosition(extendInitPos);
        sliderR.setTargetPosition(sliderR.getCurrentPosition() - 100);
        sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderR.setPower(0.6);
    }

    public void sliderL_extend() {
//        extend.setTargetPosition(extendInitPos);
        sliderL.setTargetPosition(sliderL.getCurrentPosition() + 100);
        sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderL.setPower(0.6);
    }

    public void sliderL_retract() {
//        extend.setTargetPosition(extendInitPos);
        sliderL.setTargetPosition(sliderL.getCurrentPosition() - 100);
        sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderL.setPower(0.6);
    }

    public void slider_INIT() {
        sliderL.setTargetPosition(sliderLlow);
        sliderR.setTargetPosition(sliderRlow);
    }


    //TODO: OUTARM POSITION (VERTICAL)
    public void pixelTake() {
        outL.setPosition(outL_pixPick);
        outR.setPosition(outR_pixPick);
    }

    public void pixelDropArm() {
        outL.setPosition(outL_pixDrop);
        outR.setPosition(outR_pixDrop);
    }

    //    TODO: OUTARM EXTENSION
    public void outPixInExtend() {
//        outArm_extend.setPosition(outArmExtend_pixDrop);
        double newPosition = outArm_extend.getPosition() + servoInc;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        outArm_extend.setPosition(newPosition);
        sleep(200);

    }

    public void outPixInRetract() {
//        outArm_extend.setPosition(outArmExtend_pixDrop);
        double newPosition = outArm_extend.getPosition() - servoInc;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        outArm_extend.setPosition(newPosition);
        sleep(200);

    }

    //TODO: OUTARM ROTATE
    public void armRotateleft() {
        double newPosition = outArm_rotate.getPosition() + servoInc;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        outArm_rotate.setPosition(newPosition);
        sleep(200);
    }

    public void armRotateright() {
        double newPosition = outArm_rotate.getPosition() - servoInc;
        newPosition = Math.min(1.0, Math.max(0.0, newPosition));
        outArm_rotate.setPosition(newPosition);
        sleep(200);
    }
}
