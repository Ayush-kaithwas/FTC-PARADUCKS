package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.geometry.AsymmetricMotionProfile;
@Disabled
@Config
public class ServoProfiler {
//    private AsymmetricMotionProfile servoProfile;
    public double maxVel = 500;
    public double maxAccel = 200;

    ElapsedTime timer;
    public double targetX;
    MotionProfile motionProfilex;

    public Servo s1;
    private Telemetry localTelemetry;


    public ServoProfiler(HardwareMap hardwareMap, Telemetry localTelemetry) {
        this.localTelemetry = telemetry;
        s1 = hardwareMap.get(Servo.class, "s1");
        timer = new ElapsedTime();

    }


    //Motion Profiling
    public void goTo(double x, double maxVel, double maxAccel) {
        targetX = x;
        double currentx = s1.getPosition();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentx, 0, 0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );
        timer.reset();

    }

    public void goTo(double x) {
        targetX = x;
        double currentSliderPos = s1.getPosition();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentSliderPos, 0, 0),                //current pos is passed through motionProfilex in state
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );
        timer.reset();
    }

    //update is being called in robot class as setPoint
    public double update() {
        if (motionProfilex != null) {
            MotionState xState = motionProfilex.get(timer.seconds());          //Current pos recieved as it is passed in constructor from above.

            return xState.getX();

        }
        return 0;
    }

}