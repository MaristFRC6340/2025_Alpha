// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),() -> m_driverController.getLeftY() * -1,() -> m_driverController.getLeftX() * -1).withControllerRotationAxis(m_driverController::getRightX).deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true);
  

  SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser("default");
    SmartDashboard.putData("OPMODE/autochooser",autoChooser);
    configureBindings();
    registerNamedCommands();
  }

  /**
   * 
   */
  private void configureBindings() {
    m_SwerveSubsystem.setDefaultCommand(m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity));
   
  }
  /**
   * For cleanliness, register all named commands here
   */
  private void registerNamedCommands(){

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
