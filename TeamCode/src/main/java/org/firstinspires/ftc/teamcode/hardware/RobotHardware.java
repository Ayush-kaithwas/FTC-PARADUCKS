package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;

import javax.annotation.concurrent.GuardedBy;

public class RobotHardware {
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;
    public static HardwareMap hardwareMap;
    public MecanumDrivetrain drivetrain;

    private double voltage = 12.0;
    public boolean enabled;
    private static RobotHardware instance = null;
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;

    public TwoWheelTrackingLocalizer localizer;
    public PreloadDetectionPipeline preloadDetectionPipeline;

    private AprilTagProcessor aprilTag;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // DRIVETRAIN
        this.leftRear = hardwareMap.get(DcMotorEx.class, "LRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftFront = hardwareMap.get(DcMotorEx.class, "LFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightRear = hardwareMap.get(DcMotorEx.class, "RRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightFront = hardwareMap.get(DcMotorEx.class, "RFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        drivetrain = new MecanumDrivetrain();

        this.preloadDetectionPipeline = new PreloadDetectionPipeline();

    }
    public void read() {

        if (Globals.IS_AUTO) {
//            values.put(Sensors.SensorType.POD_LEFT, podLeft.getPosition());
//            values.put(Sensors.SensorType.POD_FRONT, podFront.getPosition());
//            values.put(Sensors.SensorType.POD_RIGHT, podRight.getPosition());
        }

    }

    public void write() {

        drivetrain.write();
    }

    public void periodic() {
//        if (voltageTimer.seconds() > 5) {
//            voltageTimer.reset();
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        }


        drivetrain.periodic();
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTag != null && localizer != null) return aprilTag.getDetections();
        System.out.println("Active");
        return null;
    }
    public double getVoltage() {
        return voltage;
    }
    public double getAngle() {
        return AngleUnit.normalizeRadians(imuAngle - imuOffset);
    }


}
