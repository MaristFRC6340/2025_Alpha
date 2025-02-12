package frc.robot.commands;

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;
import java.nio.file.FileSystem;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.function.DoubleSupplier;

public class LittletonWheelRadiusCommand extends Command {
  private static final double characterizationSpeed = 0.1;
  private static final double driveRadius = Constants.SwerveConstants.kStoredRadius;

  private SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    private final DoubleSupplier gyroYawRadsSupplier =
        () -> drive.getHeading().getRadians();
    private final int omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
  
    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;
  
    private double[] startWheelPositions;
  
    private double currentEffectiveWheelRadius = 0.0;
  
    public LittletonWheelRadiusCommand(SwerveSubsystem drive, int omegaDirection) {
      this.drive = drive;
    this.omegaDirection = omegaDirection;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection * characterizationSpeed));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}