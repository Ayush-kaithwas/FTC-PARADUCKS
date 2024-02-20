package org.firstinspires.ftc.teamcode.commandbase.subsytem;

import  com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@TeleOp
@Config
public class HangerAndDrone extends SubsystemBase {
    public Servo hangL1,hangL2, drone;
    public static DcMotorEx HangerMotor;
    public static double hangLock1Init = 0.5, droneInit = 0.65, droneShoot = 0.5, droneLock = 0.8, hangLock1Open = 0.44, i = 0;
     public static double hanglock2Init=0.5, hangLock2Open=0.22;

    public static int pos = 8900;//6390; //6737

    public HangerAndDrone(HardwareMap hardwareMap, Telemetry telemetry){
        hangL1 = hardwareMap.get(Servo.class, "hangL1");
        hangL2 = hardwareMap.get(Servo.class, "hangL2");
        drone = hardwareMap.get(Servo.class, "plane");
        HangerMotor = hardwareMap.get(DcMotorEx.class, "hangerM");
        HangerMotor.setDirection(DcMotorEx.Direction.REVERSE);
        HangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HangerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HangerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void HangLock1Init(){
        hangL1.setPosition(hangLock1Init);
    }
    public void HangLock1Open(){
        hangL1.setPosition(hangLock1Open);
    }
    public void HangLock2Init(){
        hangL2.setPosition(hanglock2Init);
    }
    public void HangLock2Open(){hangL2.setPosition(hangLock2Open);}

    public void ShootDrone(){
        drone.setPosition(droneShoot);
    }
    public void LockDrone(){
        drone.setPosition(droneLock);
    }

    public  void HangerUp(){
//        HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() + 28);
        HangerMotor.setTargetPosition(8500);
        HangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
//        done = true;
    }
    public void HangerDown(){
        HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() - 30);
        HangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
    }

    public static void Hang2Pos(){
        HangerMotor.setTargetPosition(pos);
        HangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
    }
    public void HangerInc(int pos){
        HangerMotor.setTargetPosition(pos + 100);
        HangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
    }
    public void hanger_INIT(){
        HangerMotor.setTargetPosition(0);
    }
//    public void hangLock_INIT(){
//        hang.setPosition(0.8);
//    }



//    public void runOpMode(){
//        hanger = hardwareMap.get(Servo.class, "hang");
//        drone = hardwareMap.get(Servo.class, "plane");
//        HangerMotor = hardwareMap.get(DcMotor.class, "hangerMotor");
//
//        HangerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        HangerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        HangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        drone.setPosition(droneInit);
//        hanger.setPosition(hangerInit);
//        waitForStart();
//
//        while(opModeIsActive()){
//            if(gamepad1.dpad_up){
//                hanger.setPosition(hanger.getPosition() + 0.001);
//            }
//            if(gamepad1.dpad_down){
//                hanger.setPosition(hanger.getPosition() - 0.001);
//            }
//
//            if(gamepad1.dpad_left){
//                drone.setPosition(drone.getPosition() + 0.001);
//            }
//            if(gamepad1.dpad_right){
//                drone.setPosition(drone.getPosition() - 0.001);
//            }
//
//            if(gamepad1.x){
//                drone.setPosition(droneShoot);
//            }
//            if(gamepad1.b){
//                while(i < hangShoot){
//                    hanger.setPosition(hanger.getPosition() + 0.01);
//                }
//                //hanger.setPosition(hangShoot);
//            }
//
//            if(gamepad1.y){
//                //int pos = HangerMotor.getCurrentPosition() + 20;
//                HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() - 11488);
//                HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                HangerMotor.setPower(1);
//            }
//            if(gamepad1.a){
//              //  int pos = HangerMotor.getCurrentPosition() - 20;
//                HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() +  11488);
//                HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                HangerMotor.setPower(1);
//            }
//            telemetry.addData("Hanger Position:", hanger.getPosition());
//            telemetry.addData("Drone Position:", drone.getPosition());
//            telemetry.addData("HangerMotor pos: ", HangerMotor.getCurrentPosition());
//            telemetry.update();
//        }
//    }

}
