// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
      public static final String name = "tempalte";
      public static  Rotation3d robotToCamTransform = new Rotation3d(0, Units.degreesToRadians(18), 0);
      public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(-4.628),
      Units.inchesToMeters(-10.687),
      Units.inchesToMeters(16.129));
      public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(4, 4, 8);
      public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
    }


    


  }

  public static class DriveConstants{

    
  }

public static final String ElevatorConstants = null;
public static final String CoralConstants = null;
}

