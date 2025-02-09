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
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraTemplate;

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
  @Logged

public class VisionSubsystem
{

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout  = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  
  
  /**
   * Count of times that the odom thinks we're more than 10meters away from the april tag.
   */
  private             double              longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private             Supplier<Pose2d>    currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private             Field2d             field2d;


  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public VisionSubsystem(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;

    
  }


  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   * Method is Called by Swerve Periodic
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    for (Cameras camera : Cameras.values())
    {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),pose.timestampSeconds,camera.curStdDevs);
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
    return camera.getEstimatedGlobalPose();
  }


  

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField()
  {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values())
    {
      if (!c.resultsList.isEmpty())
      {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets())
        {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * OBJECT WHICH HOLDS ALL INFORMATION ABOUT EACH INDIVIDUAL CAMERA
   * 
   * Camera Enum to select each camera
   */
  enum Cameras
  {
    
    /**
     * Add Cameras Here; Try and put Constants in Vision SubsystemClass maybe in its Own Constants Class
     */
    CENTER_CAM(CameraTemplate.name,CameraTemplate.robotToCamTransform,CameraTemplate.robotToCamTranslation,CameraTemplate.singleTagStdDevs, CameraTemplate.multiTagStdDevsMatrix);

   
    /**
     * Camera instance for comms.
     */
    public final  PhotonCamera                 camera;
    /**
     * Pose estimator for camera.
     */
    public final  PhotonPoseEstimator          poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d                  robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public        Matrix<N3, N1>               curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public        PhotonCameraSim              cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary queries.
     */
    public        List<PhotonPipelineResult>   resultsList       = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private  double  lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);//latency alert

      camera = new PhotonCamera(name);//create actual photon camera

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html DEF MAKE SURE TO DOUBLE CHECK THIS WHEN ADDING TRANSLATIONS
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(VisionSubsystem.fieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      
      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
    }

    



    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
     */
    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp    = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime        = Milliseconds.of(VisionConstants.debouncetimeMilli).in(Seconds);
      for (PhotonPipelineResult result : resultsList)
      {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
    //   if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
    //       (currentTimestamp - lastReadTimestamp) >= debounceTime)
    if(true)//not sure if yagsl has fixed this yet
      {
        resultsList = camera.getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty())
        {
          updateEstimatedGlobalPose();
        }
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
     * per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
     * estimation.
     */
    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else
      {
        // Pose present. Start running Heuristic
        var    estStdDevs = singleTagStdDevs;
        int    numTags    = 0;
        double avgDist    = 0;

        boolean reef = false;
        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
            continue;
          }
          double distance = tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          if(Constants.FieldPositions.isReefID(tgt.getFiducialId()) && Math.abs(distance)<1) {
            reef=true;
          }
          numTags++;
          avgDist += distance;
              
        }

        if (numTags == 0)
        {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else
        {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if(reef) {
            estStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
          }
          else if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          else if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }


  }

}