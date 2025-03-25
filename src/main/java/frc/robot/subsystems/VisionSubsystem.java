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
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraTemplate;
import frc.robot.Constants.VisionConstants.ReefCamera;

import java.awt.Desktop;
import java.text.FieldPosition;
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
  public static  AprilTagFieldLayout fieldLayout;
  
  
 
  private             Field2d             field2d;

  //new camera stuff

  private PhotonCamera leftReefCamera;
  private PhotonPoseEstimator leftPoseEstimator;
  private int leftLatestID;
  private Pose2d leftCameraEstimatedRobotToCam;




  private PhotonCamera rightReefCamera;
  private PhotonPoseEstimator rightPoseEstimator;
  private int rightLatestID;
  private Pose2d rightCameraEstimatedRobotToCam;






  private PhotonCamera reefCamera;
  private Optional<Pose2d> lastCalculatedDist;
  private PhotonPoseEstimator poseEstimator;
  private int latestID;
  private Pose2d reefDstPose;

  StructPublisher<Pose2d> reefTagDisp = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Vision/RobotToTag", Pose2d.struct).publish();
  StructPublisher<Pose2d> estimatedCaemraPose = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Vision/estimatedCameraPose", Pose2d.struct).publish();

  public VisionSubsystem( Field2d field)
  {
    this.field2d = field;
    try{
      //= new AprilTagFieldLayout(Filesystem.getDeployDirectory()+"2025-reefscape-welded.json"); // k2025Reefscape
    }
    catch(Exception e){
      System.out.println("Failed");

    }
    fieldLayout= AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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
      reefDstPose = pose.get();
      reefTagDisp.set(reefDstPose);

      SmartDashboard.putBoolean("Subsystem/ALIGNED", 
      (((Math.abs(reefDstPose.getX()- Constants.VisionConstants.leftAlignmentX)<VisionConstants.xTolerance) 
      && (Math.abs(reefDstPose.getY()-Constants.VisionConstants.leftAlignmentY)<VisionConstants.yTolerance)
      && (Math.abs(reefDstPose.getRotation().getRadians()-Constants.VisionConstants.thetaAlignment)<VisionConstants.thetaTolerance))||
      ((Math.abs(reefDstPose.getX()- Constants.VisionConstants.rightAlignmentX)<VisionConstants.xTolerance) 
      && (Math.abs(reefDstPose.getY()-Constants.VisionConstants.rightAlignmentY)<VisionConstants.yTolerance)
      && (Math.abs(reefDstPose.getRotation().getRadians()-Constants.VisionConstants.thetaAlignment)<VisionConstants.thetaTolerance))
      ));

      SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN", latestID!=-1 && Constants.FieldPositions.isReefID(latestID) && pose.get().getX()<Constants.VisionConstants.maxAlignmentDistance);

    }
    else{
      SmartDashboard.putBoolean("Subsystem/Vision_CAN_ALIGN", false);
      SmartDashboard.putBoolean("Subsystem/ALIGNED", false);

    }
    
    var result = reefCamera.getLatestResult();
    if(result!=null && result.hasTargets()) {
      latestID = result.getBestTarget().getFiducialId();
    }
    else {
      latestID = -1;
    }
    
  }

  public int getBestTag(Pose2d pose, List<PhotonTrackedTarget> targetsSeen){

    int bestId=0;
    double bestDistance = Double.MAX_VALUE;
    for(PhotonTrackedTarget t : targetsSeen) {
      if(!Constants.FieldPositions.isReefID(t.getFiducialId())) continue;
        double distance = Math.abs(t.getBestCameraToTarget().getY());
      if(distance<bestDistance) {
        bestId=t.getFiducialId();
        bestDistance=distance;
      }
    }
    return bestId;
  }

  //Both Cameras
  public void updateRobotToTagTransformForBothCameras() {
    // Get the latest result from the camera
    PhotonPipelineResult leftResult = leftReefCamera.getLatestResult();
    PhotonPipelineResult rightResult = rightReefCamera.getLatestResult();

    if(rightResult!=null && rightResult.hasTargets()){
      Optional<EstimatedRobotPose> rightEstimatedPose = poseEstimator.update(rightResult);
      if(rightEstimatedPose.isPresent()){
          EstimatedRobotPose estimatedPose = rightEstimatedPose.get(); 
          estimatedCaemraPose.set(estimatedPose.estimatedPose.toPose2d());
         int bestId  = getBestTag(estimatedPose.estimatedPose.toPose2d(), rightResult.getTargets());

          SmartDashboard.putNumber("Subsystem/Vision/BestReefId", bestId);

          // Retrieve the pose of the detected tag from the field layout
          Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(bestId);

          // Ensure the tag pose is available
          if (tagPoseOptional.isPresent()) {
              Pose3d tagPose = tagPoseOptional.get();
              // Compute the robot's pose relative to the tag
              Pose2d robotPose = estimatedPose.estimatedPose.toPose2d();
              Pose2d tagPose2d = tagPose.toPose2d();
              Pose2d robotInTagSpace = robotPose.relativeTo(tagPose2d);
              rightCameraEstimatedRobotToCam = robotInTagSpace;
          }
    }
    else{
      rightCameraEstimatedRobotToCam = null;
    }
   
  } else{
    rightCameraEstimatedRobotToCam = null;
  }
}
//do the same for left

 



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
            // double rotation = estimatedPose.estimatedPose.toPose2d().getRotation().getRadians();
            double y = estimatedPose.estimatedPose.toPose2d().getRotation().getRadians();

            // Get the ID of the first detected tag
            //int bestId = getClosestReefSide(estimatedPose.estimatedPose.toPose2d());
            
            estimatedCaemraPose.set(estimatedPose.estimatedPose.toPose2d());
            int bestId=0;
            double bestDistance = Double.MAX_VALUE;
            for(PhotonTrackedTarget t : result.getTargets()) {
              if(!Constants.FieldPositions.isReefID(t.getFiducialId())) continue;
              // double distance = Math.abs(t.getYaw()-rotation);
                double distance = Math.abs(t.getBestCameraToTarget().getY());

              if(distance<bestDistance) {
                bestId=t.getFiducialId();
                bestDistance=distance;
              }
            }
            //int tagID = result.getBestTarget().getFiducialId();
            SmartDashboard.putNumber("Subsystem/Vision/BestReefId", bestId);
            // Retrieve the pose of the detected tag from the field layout
            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(bestId);

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

    /**
   * Find the closest reef side to the estimated robot pose
   * @return
   */
  public static int getClosestReefSide (Pose2d pose) {
    var alliance = DriverStation.getAlliance();
    boolean red = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    Translation2d reefCenter = red ? Constants.FieldPositions.RED_REEF_CENTER : Constants.FieldPositions.BLUE_REEF_CENTER;
    Translation2d reefRelativePose = new Translation2d(pose.getX()-reefCenter.getX(), pose.getY()-reefCenter.getY());
    double angleToReef = 90;
    if(reefRelativePose.getY()!=0)
    {
      angleToReef = Math.toDegrees(Math.atan(reefRelativePose.getY()/reefRelativePose.getX()));
    }
    System.out.println(angleToReef);
    //TODO: Replace with checking which side we are on
    if(!red) {
      //BLUE
      if(reefRelativePose.getY()>=0) {
        if(angleToReef>=0 && angleToReef <=30) {
          return 21;
        }
        if(angleToReef>=30 && angleToReef<=90) {
          return 20;
        }
        if(angleToReef<=-30 && angleToReef>=-90) {
          return 19;
        }
        if(angleToReef >=-30 && angleToReef<=0) {
          return 18;
        }
      }
      else {
          if(angleToReef>=0 && angleToReef <=30) {
            return 18;
          }
          if(angleToReef>=30 && angleToReef<=90) {
            return 17;
          }
          if(angleToReef<=-30 && angleToReef>=-90) {
            return 22;
          }
          if(angleToReef >=-30 && angleToReef<=0) {
            return 21;
          }
          
      }
    } else {
      if(reefRelativePose.getY()>=0) {
        if(angleToReef>=0 && angleToReef <=30) {
          return 7;
        }
        if(angleToReef>=30 && angleToReef<=90) {
          return 8;
        }
        if(angleToReef<=-30 && angleToReef>=-90) {
          return 9;
        }
        if(angleToReef >=-30 && angleToReef<=0) {
          return 10;
        }
      }
      else {
          if(angleToReef>=0 && angleToReef <=30) {
            return 10;
          }
          if(angleToReef>=30 && angleToReef<=90) {
            return 11;
          }
          if(angleToReef<=-30 && angleToReef>=-90) {
            return 6;
          }
          if(angleToReef >=-30 && angleToReef<=0) {
            return 7;
          }
          
      }
    }
    return -1;
  }

  
}
