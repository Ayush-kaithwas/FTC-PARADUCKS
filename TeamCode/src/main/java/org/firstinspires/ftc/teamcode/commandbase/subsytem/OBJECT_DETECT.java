package org.firstinspires.ftc.teamcode.commandbase.subsytem;

import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;

public class OBJECT_DETECT {
    public static VisionPortal visionPortal;
    public OpenCvCamera leftCamera;
    public static VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

}
