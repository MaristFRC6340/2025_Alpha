package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraTemplate;
import frc.robot.Constants.VisionConstants.ReefCamera;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */

public class VisionSubsystem extends SubsystemBase
{

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout  = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded); // k2025Reefscape
  
  
 
  private             Field2d             field2d;

  private PhotonCamera reefCamera;
  private PhotonPipelineResult latestSuccessfulResult;

  StructPublisher<Transform3d> reefTagDisp = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Vision/reefToCameraTransform", Transform3d.struct).publish();

  public VisionSubsystem( Field2d field)
  {
    this.field2d = field;
    reefCamera = new PhotonCamera(Constants.VisionConstants.ReefCamera.name);
    // Constants.VisionConstants.ReefCamera.stdDevsMap.put();


    
  }

  @Override
  public void periodic(){

    updateResults();
    reefTagDisp.set(getReefToCamera());
    SmartDashboard.putNumber("SmartDashboard/Subsystem/Vision/rotation", Math.toDegrees(getReefToCamera().getRotation().getAngle()));
  }


  public void updateResults() {
    PhotonPipelineResult currentResult = null;
    var unreadResults = reefCamera.getAllUnreadResults();
    if(unreadResults.size()>0) {
      currentResult = unreadResults.get(0);
    }
    if(currentResult!=null && currentResult.hasTargets() && currentResult.getBestTarget().poseAmbiguity<.3) {
      latestSuccessfulResult = currentResult;
    }
  }

  public Transform3d getReefToCamera() {
    if(latestSuccessfulResult != null && Constants.FieldPositions.isReefID(latestSuccessfulResult.getBestTarget().getFiducialId())) {
      return latestSuccessfulResult.getBestTarget().getBestCameraToTarget();
    }
    return new Transform3d();
  }

}