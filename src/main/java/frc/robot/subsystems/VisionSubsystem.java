package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
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
import edu.wpi.first.wpilibj.Timer;
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
  private Optional<Pose2d> lastCalculatedDist;
  private PhotonPoseEstimator poseEstimator;
  private int latestID;

  StructPublisher<Pose2d> reefTagDisp = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Vision/RobotToTag", Pose2d.struct).publish();

  public VisionSubsystem( Field2d field)
  {
    this.field2d = field;
    reefCamera = new PhotonCamera(Constants.VisionConstants.ReefCamera.name);
    // Constants.VisionConstants.ReefCamera.stdDevsMap.put();
    poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, new Transform3d(Constants.VisionConstants.ReefCamera.robotToCamTranslation, Constants.VisionConstants.ReefCamera.robotToCamTransform));
    lastCalculatedDist = Optional.empty();
    
  }

  @Override
  public void periodic(){
    var pose = getRobotInTagSpace();
    SmartDashboard.putBoolean("Subsystem/posePresent", pose.isPresent());

    if(pose!=null && pose.isPresent())  {
      reefTagDisp.set(pose.get());
      SmartDashboard.putBoolean("Subsystem/idCheck", latestID!=-1 && Constants.FieldPositions.isReefID(latestID));
      SmartDashboard.putBoolean("Subsystem/posCHeck",  pose.get().getX()<Constants.VisionConstants.maxAlignmentDistance );


      SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN", latestID!=-1 && Constants.FieldPositions.isReefID(latestID) && pose.get().getX()<Constants.VisionConstants.maxAlignmentDistance);

    }
    else{
      SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN", false);

    }
    
    var result = reefCamera.getLatestResult();
    if(result!=null && result.hasTargets()) {
      latestID = result.getBestTarget().getFiducialId();
    }
    else {
      latestID = -1;
    }
    
  }


 



  public Optional<Pose2d> getRobotInTagSpace() {
    // Get the latest result from the camera
    PhotonPipelineResult result = reefCamera.getLatestResult();
    
    // Check if any targets are detected
    if (result!=null && result.hasTargets()) {
        // Get the current timestamp

        // Update the pose estimator with the latest result
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update(result);

        // Check if a pose was estimated
        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();

            // Get the ID of the first detected tag
            int tagID = result.getBestTarget().getFiducialId();

            // Retrieve the pose of the detected tag from the field layout
            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(tagID);

            // Ensure the tag pose is available
            if (tagPoseOptional.isPresent()) {
                Pose3d tagPose = tagPoseOptional.get();

                // Compute the robot's pose relative to the tag
                Pose2d robotPose = estimatedPose.estimatedPose.toPose2d();
                Pose2d tagPose2d = tagPose.toPose2d();
                Pose2d robotInTagSpace = robotPose.relativeTo(tagPose2d);
                lastCalculatedDist = Optional.of(robotInTagSpace);

                // Return the robot's pose in tag space
                return Optional.of(robotInTagSpace);
            }
        }
    }

    // Return empty if no valid pose could be estimated
    return lastCalculatedDist;
  }

  public int getLatestID() { 
    return latestID;
  }
  
}
