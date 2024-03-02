// Originally: https://github.com/STMARobotics/swerve-test/blob/main/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java

package frc.robot.subsystems;

//import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.VisionCamera;

public class PoseEstimatorSubsystem extends SubsystemBase implements Supplier<Pose2d> {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimatorSubsystem( DrivetrainSubsystem drivetrainSubsystem) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      /* \
      // TODO: 2023 uses an absolute coordinate system, so this has a 50% chance of rotating all coordinates 180deg!
      var alliance = DriverStation.getAlliance();
      layout.setOrigin(alliance == Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
          */
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator(
        Constants.KINEMATICS,
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(2.5, 2.5, Rotation2d.fromDegrees(90)),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    poseEstimator.resetPosition(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getModulePositions(),
      new Pose2d());
    
    tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  
  @Override
  public void periodic() {

    for (VisionCamera cam : Constants.VisionCameras){
        // Update pose estimator with the best visible target
        var pipelineResult = cam.pvCam.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
            previousPipelineTimestamp = resultTimestamp;
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            // Get the tag pose from field layout - consider that the layout will be null if it failed to load
            Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
                var targetPose = tagPose.get();
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
                var visionMeasurement = camPose.transformBy(cam.CameraToRobot);
                //SmartDashboard.putString(String.valueOf(fiducialId), String.valueOf(resultTimestamp));
                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }
    }
    
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
  }

//   static Pose2d start_pose = new Pose2d(5, 5, new Rotation2d(0));
//   static Pose2d new_pose = start_pose;
  
    @Override
    public void simulationPeriodic() {
        // new_pose = new_pose.transformBy(new Transform2d(new Translation2d(), new Rotation2d(0.002)));
        // poseEstimator.resetPosition(
        //     drivetrainSubsystem.getGyroscopeRotation(),
        //     drivetrainSubsystem.getModulePositions(),
        //     new_pose);
    }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }



  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getCurrentRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      drivetrainSubsystem.getGyroscopeRotation(),
      drivetrainSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving. This assumes that estimating field position
   * is not needed, once a driver has taken control of the robot.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  @Override
  public Pose2d get() {
      return getCurrentPose();
  }

}
