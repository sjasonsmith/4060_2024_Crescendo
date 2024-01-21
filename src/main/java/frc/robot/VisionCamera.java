package frc.robot;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionCamera {
    public String Name;
    public String NetworkTable;
    public Transform3d CameraToRobot;
    public PhotonCamera pvCam;

    public VisionCamera(String Name, String NetworkTable, Transform3d CameraToRobot, PhotonCamera pvCam){
        this.Name = Name;
        this.NetworkTable = NetworkTable;
        this.CameraToRobot = CameraToRobot;
        if (pvCam != null){
            this.pvCam = pvCam;
        }
    }

    public Transform3d RobotToCamera(){
        return this.CameraToRobot.inverse();
    }
      
}