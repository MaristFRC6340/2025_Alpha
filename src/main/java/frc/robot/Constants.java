// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double DEADBAND = .1;

    public static int kActuatorControllerPort = 1;
  }

  public static class RobotConstants{
    //TODO update these: 
    public static final double MAX_SPEED = Units.feetToMeters(14.5);//default from YAGSL
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound DEFAULT FROM YAGSL
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);//default fomr YAGSL
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag DEFAULT FROM YAGSL

  }
  public static final class VisionConstants{
    public static final  double maximumAmbiguity                = 0.25;
    public static final double debouncetimeMilli = 15;

    public static final class CameraTemplate{
      public static final String name = "tempalte"; // tempalte?
      public static  Rotation3d robotToCamTransform = new Rotation3d(0, Units.degreesToRadians(18), 0);
      public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(-4.628),
      Units.inchesToMeters(-10.687),
      Units.inchesToMeters(16.129));
      public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(4, 4, 8);
      public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
    }
  }

  // goal: find a good config (maybe online?) and stick with it
  public static class ElevatorConstants {
    /* 
     * Front Left Elevator: 30
     * Front Right Elevator: 31
     * Rear Left Elevator: 32
     * Rear Right Elevator: 33
    */
    public static final int kLeftID = 30;
    public static final int kRightID = 31;

    public static final double kGearRatio = 9;

    public static final double kMin = 0;
    public static final double kMax = 3;


    private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
    .withMotionMagicAcceleration(10)
    .withMotionMagicCruiseVelocity(5)
    .withMotionMagicExpo_kA(0)
    .withMotionMagicExpo_kV(0);

    private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
    .withSensorToMechanismRatio(kGearRatio);

    private static final CurrentLimitsConfigs kCurrentLimitConfigs = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(40);

    private static final Slot0Configs kSlot0Configs = new Slot0Configs()
    .withKA(0)
    .withKG(0)
    .withKS(0)
    .withKV(0)
    .withKP(1)
    .withKI(0)
    .withKD(0);

    public static final TalonFXConfiguration kLeftConfig = new TalonFXConfiguration()
    .withSlot0(kSlot0Configs)
    .withCurrentLimits(kCurrentLimitConfigs)
    .withFeedback(kFeedbackConfigs)
    .withMotionMagic(kMagicConfigs);

    public static final TalonFXConfiguration kRightConfig = new TalonFXConfiguration()
    .withSlot0(kSlot0Configs)
    .withCurrentLimits(kCurrentLimitConfigs)
    .withFeedback(kFeedbackConfigs)
    .withMotionMagic(kMagicConfigs);
  }

  public static class CoralConstants {
    public static final int kTopID = 40;
    public static final SparkMaxConfig kTopConfig = new SparkMaxConfig(); // :/
    public static final int kBottomID = 41;
    public static final SparkMaxConfig kBottomConfig = new SparkMaxConfig();

    // using same speed for top and bottom
    public static double kOuttakeMotorSpeed = 0.8; // speed values between -1.0 and 1.0
  }

  public static class ClimberConstants{
    public static final int kClimberId = 60;
    public static final int kServoID = 1;
    public static final double kRatchetOff = 0;
    public static final double kRatchetOn = 1;

    private static final Slot0Configs kSlot0Configs = new Slot0Configs()
    .withKP(1);

    public static final TalonFXConfiguration kClimberConfig= new TalonFXConfiguration()
    .withSlot0(kSlot0Configs);
  }

  public static class HuggerConstants {
    public static final int kLeftID = 50;
    public static final int kRightID = 51;
    
    


    public static final int kPivotID = 52;
    public static final double kGPivot = 0.1;
    public static final double kSPivot = 0;
    public static final double kVPivot = 0;
    public static final double kPPivot = 1;
    public static final double kIPivot = 0;
    public static final double kDPivot = 0;
  }

  public static class SwerveConstants {
    public static final double kPX = 1;
    public static final double kPY = 1;
    public static final double kPTheta = 1;
    public static final double kXTolerance = 0; //Meters
    public static final double kYTolerance = 0; //Meters
    public static final double kThetaTolerance = Math.toRadians(15);
    public static double kStoredRadius = 4; // to be configured later
  }

  public static class FieldPositions {
    //Blue
    public static final Pose2d L17 = new Pose2d(3.82, 2.96, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d L18 = new Pose2d(3.22, 4.08, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d L19 = new Pose2d(3.89, 5.16, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d L20 = new Pose2d(5.16, 5.12, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d L21 = new Pose2d(5.76, 4.00, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d L22 = new Pose2d(5.09, 2.92, new Rotation2d(Math.toRadians(210)));

    public static final Pose2d R17 = new Pose2d(4.32, 2.67, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d R18 = new Pose2d(3.22, 3.51, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d R19 = new Pose2d(3.40, 4.87, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d R20 = new Pose2d(4.67, 5.40, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d R21 = new Pose2d(5.76, 4.57, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d R22 = new Pose2d(5.59, 3.20, new Rotation2d(Math.toRadians(210)));

    //Red
    public static final Pose2d L6 = new Pose2d(13.65, 2.92, new Rotation2d(Math.toRadians(210)));
    public static final Pose2d L7 = new Pose2d(14.32, 4.00, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d L8 = new Pose2d(13.72, 5.12, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d L9 = new Pose2d(12.45, 5.16, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d L10 = new Pose2d(11.78, 4.08, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d L11 = new Pose2d(12.38, 2.96, new Rotation2d(Math.toRadians(150)));

    public static final Pose2d R6 = new Pose2d(14.14, 3.20, new Rotation2d(Math.toRadians(210)));
    public static final Pose2d R7 = new Pose2d(14.32, 4.57, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d R8 = new Pose2d(13.22, 5.40, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d R9 = new Pose2d(11.95, 4.87, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d R10 = new Pose2d(11.78, 3.51, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d R11 = new Pose2d(12.87, 2.67, new Rotation2d(Math.toRadians(150)));

    public static final List<Pose2d> kReefPoses=Arrays.asList(
    L17, L18, L19, L20, L21, L22, R17, R18, R19, R20, R21, R22, L6, L7, L8, L9, L10, L11, R6, R7, R8, R9, R10, R11
    );


    public static final int [] kReefIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static boolean isReefID(int id) {
      for(int i: kReefIDs) {
        if(id==i) return true;
      }
      return false;
    }
  }


}

