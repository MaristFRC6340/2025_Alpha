// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import java.io.File;
import java.io.IOException;
import java.util.AbstractSet;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive         swerveDrive;
  /**
   * AprilTag field layout.
   */
  public VisionSubsystem vision;
  
  private PIDController xController = new PIDController(Constants.SwerveConstants.kPX, .1, 0);
  private PIDController yController = new PIDController(Constants.SwerveConstants.kPY, .1, 0);
  private PIDController thetaController = new PIDController(Constants.SwerveConstants.kPTheta*2, .1, 0);

 

  StructPublisher<Pose2d> finalPoseEstimate = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Swerve/finalPoseEstimate", Pose2d.struct).publish();
  StructPublisher<ChassisSpeeds> curChassisSpeed = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/Subsystem/Swerve/curChassisSpeeds", ChassisSpeeds.struct).publish();
  StructArrayPublisher<SwerveModuleState> swerveModuleState = NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/Subsystem/Swerve/curModuleStates", SwerveModuleState.struct).publish();
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.RobotConstants.MAX_SPEED, new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), new Rotation2d(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//TODO set this to true maybe? // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,true,0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    setupPhotonVision();
    setupPathPlanner();

    xController.setTolerance(Constants.SwerveConstants.kXTolerance);
    yController.setTolerance(Constants.SwerveConstants.kYTolerance);
    thetaController.setTolerance(Constants.SwerveConstants.kThetaTolerance);

    SmartDashboard.putNumber("Subsystem/Vision/xSetPoint", VisionConstants.rightAlignmentX);
    SmartDashboard.putNumber("Subsystem/Vision/ySetPoint", VisionConstants.rightAlignmentY);


  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new VisionSubsystem( swerveDrive.field);
  }

  @Override
  public void periodic()
  {

    
    // When vision is enabled we must manually update odometry in SwerveDrive
      swerveDrive.updateOdometry();
      //vision.updatePoseEstimation(swerveDrive);
      SmartDashboard.putNumber("Subsystem/Swerve/FlipDirection", flipDirection());
      SmartDashboard.putBoolean("Subsystem/Swerve/ShouldFlip", isRedAlliance());
    //these should also be printed out by Yagsl's logging, but I wanted to test out using structu publishers so that we can produe this logging fi we ever abandon yagsl
      finalPoseEstimate.set(getPose());
      curChassisSpeed.set(swerveDrive.getRobotVelocity());
      swerveModuleState.set(swerveDrive.getStates());

     // SmartDashboard.putData("Subsystem/Swerve/currentCommand", this.getCurrentCommand());
      //SmartDashboard.putData("Subsystem/Swerve/defaultCommand", this.getDefaultCommand());


    
  }

  

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(4.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            return isRedAlliance();
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }



  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command pathFindToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }



  public int flipDirection() {
    return isRedAlliance() ? -1 : 1;
  }

  /**
   * Command to characterize the robot drive motors using SysId
   * TODO: cusomtize this for sysidroutine test
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);

  }

  public SysIdRoutine getRoutine(){
    return SwerveDriveTest.setDriveSysIdRoutine(
      new Config(),
      this, swerveDrive, 12, true);

  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public Command driveRobotCentric(Supplier<ChassisSpeeds> speeds){
    return this.run(()->swerveDrive.drive(speeds.get()));
  }



  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  
  

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    //var alliance = DriverStation.getAlliance();
    return false;
    //return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }


  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.RobotConstants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.RobotConstants.MAX_SPEED);
  }


  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }


  /**
   * Spins the robot in place to find wheel radius
   * @param input
   */
  public void runWheelRadiusCharacterization(double input) {
    swerveDrive.drive(new ChassisSpeeds(0, 0, input));
  }

  public double [] getWheelRadiusCharacterizationPosition() {
    double [] out = new double [4];
    
    SwerveModulePosition [] positions = swerveDrive.getModulePositions();
    for(int i = 0; i<4; i++) {
      //current position in radians = distancemeters/(radius*2*pi)*2*pi=distancemeters/radius
      out[i]=positions[i].distanceMeters/(Constants.SwerveConstants.kStoredRadius*2*Math.PI)*2*Math.PI; //CHANGE TO WHATEVER STORED RADIUS IS
    }
    return out;
  }

  /**
   * Drives to a given pose using pid controllers
   * @param pose Target pose to go to
   * @return driving Command
   */
  public Command driveToPose(Pose2d pose) {
    return run(() -> {
      double xPower = -xController.calculate(pose.getX(), swerveDrive.getPose().getX());
      double yPower = -yController.calculate(pose.getY(), swerveDrive.getPose().getY());
      double thetaPower = -thetaController.calculate(pose.getRotation().getRadians(), swerveDrive.getPose().getRotation().getRadians());
      this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      new ChassisSpeeds(xPower, yPower, thetaPower), getHeading()
      ));
    }
    ).until(() -> {
        return Math.abs(pose.getX()-swerveDrive.getPose().getX())<=Constants.SwerveConstants.kXTolerance &&
        Math.abs(pose.getY()-swerveDrive.getPose().getY())<=Constants.SwerveConstants.kYTolerance &&
        Math.abs(pose.getRotation().getDegrees()-swerveDrive.getPose().getRotation().getDegrees())<Constants.SwerveConstants.kThetaTolerance;
    });
}

public Command driveToRobotRelativeTransform(Supplier<Transform3d> transformSupplier, boolean left) {
  return run(() -> {
    Transform3d robotToCamera = new Transform3d(0,0,0, Constants.VisionConstants.ReefCamera.robotToCamTransform);
    Rotation3d rotated = transformSupplier.get().getRotation().rotateBy(new Rotation3d(0,0,Math.toRadians(-30)));
    Pose3d disp = new Pose3d().transformBy(transformSupplier.get());
    double angle = Math.toRadians(45)+disp.getRotation().getZ()-Math.toRadians(135);

    //reefToCamera + robottoCamera^-1 = reef to Robot
    double gigaX = disp.getY()*Math.sin(angle)+disp.getX()*Math.cos(angle);
    double gigaY = disp.getX()*Math.sin(angle)-disp.getY()*Math.cos(angle); //Chatgpt did this idk what it does
    double xPower = xController.calculate(gigaX, left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX);
    double yPower = yController.calculate(gigaY, left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY);
    double thetaPower = thetaController.calculate(disp.getRotation().getZ(), Math.toRadians(135));
    this.drive(new ChassisSpeeds(-xPower, yPower, -thetaPower));
    SmartDashboard.putNumber("Subsystem/Vision/xActualPose", disp.getX());
    SmartDashboard.putNumber("Subsystem/Vision/yActualPose", disp.getY());
    SmartDashboard.putNumber("Subsystem/Vision/ThetaActual", disp.getRotation().getZ());
    SmartDashboard.putNumber("Subsystem/Vision/AdjustedY", disp.getX()*Math.sin(angle)-disp.getY()*Math.cos(angle));
    SmartDashboard.putNumber("Subsystem/Vision/AdjustedX", disp.getY()*Math.sin(angle)+disp.getX()*Math.cos(angle));
    SmartDashboard.putString("AlignmentState","Enabled");
  }).andThen(new InstantCommand(()->SmartDashboard.putString("AlignmentState","Disabled")));
}


}