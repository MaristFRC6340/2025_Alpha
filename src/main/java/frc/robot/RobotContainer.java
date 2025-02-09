// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HuggerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
  private final   HuggerSubsystem m_HuggerSubsystem = new HuggerSubsystem();
  //private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_actuatorController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //Buttons!
  private Trigger actuatorA = m_actuatorController.a();
  private Trigger actuatorB = m_actuatorController.b();
  private Trigger actuatorX = m_actuatorController.x();
  private Trigger actuatorY = m_actuatorController.y();
  private Trigger actuatorL = m_actuatorController.leftBumper();
  private Trigger actuatorR = m_actuatorController.rightBumper();
  private Trigger actuatorStart = m_actuatorController.start();
  private Trigger actuatorBack = m_actuatorController.back();
  private Trigger actuatorLTrigger = m_actuatorController.leftTrigger(.05);
  private Trigger actuatorRTrigger = m_actuatorController.rightTrigger(.05);
  private Trigger actuatorLStick = m_actuatorController.leftStick();
  private Trigger actuatorRStick = m_actuatorController.rightStick();
  private Trigger actuatorDpadUp = m_actuatorController.povUp();
  private Trigger actuatorDpadRight = m_actuatorController.povRight();
  private Trigger actuatorDpadDown = m_actuatorController.povDown();
  private Trigger actuatorDpadLeft = m_actuatorController.povLeft();

  private Trigger driverA = m_driverController.a();
  private Trigger driverB = m_driverController.b();
  private Trigger driverX = m_driverController.x();
  private Trigger driverY = m_driverController.y();
  private Trigger driverL = m_driverController.leftBumper();
  private Trigger driverR = m_driverController.rightBumper();
  private Trigger driverStart = m_driverController.start();
  private Trigger driverBack = m_driverController.back();
  private Trigger driverLTrigger = m_driverController.leftTrigger(.05);
  private Trigger driverRTrigger = m_driverController.rightTrigger(.05);
  private Trigger driverLStick = m_driverController.leftStick();
  private Trigger driverRStick = m_driverController.rightStick();
  private Trigger driverDpadUp = m_driverController.povUp();
  private Trigger driverDpadRight = m_driverController.povRight();
  private Trigger driverDpadDown = m_driverController.povDown();
  private Trigger driverDpadLeft = m_driverController.povLeft();
  

  //SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),() -> m_driverController.getLeftY() * -1,() -> m_driverController.getLeftX() * -1).withControllerRotationAxis(m_driverController::getRightX).deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true);
  

  SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
   // autoChooser = AutoBuilder.buildAutoChooser("default");
    //SmartDashboard.putData("OPMODE/autochooser",autoChooser);
    configureBindings();
    registerNamedCommands();
  }

  /**
   * 
   */
  private void configureBindings() {
    //m_SwerveSubsystem.setDefaultCommand(m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity));
    
    actuatorLTrigger.whileTrue(m_CoralSubsystem.getSetSpeedCommand(.7));
    actuatorRTrigger.whileTrue(m_CoralSubsystem.getShadowTechniqueCommand(.5));
    actuatorB.onTrue(m_HuggerSubsystem.setPosition(-3));
    actuatorX.onTrue(m_HuggerSubsystem.setPosition(0));

    
      // m_driverController.y().whileTrue(m_elevator.getRoutine().quasistatic(Direction.kForward));
      // m_driverController.a().whileTrue(m_elevator.getRoutine().quasistatic(Direction.kReverse));
      // m_driverController.b().whileTrue(m_elevator.getRoutine().dynamic(Direction.kForward));
      // m_driverController.x().whileTrue(m_elevator.getRoutine().dynamic(Direction.kReverse));
      //  m_driverController.y().whileTrue(m_SwerveSubsystem.getRoutine().quasistatic(Direction.kForward));
      // m_driverController.a().whileTrue(m_SwerveSubsystem.getRoutine().quasistatic(Direction.kReverse));
      // m_driverController.b().whileTrue(m_SwerveSubsystem.getRoutine().dynamic(Direction.kForward));
      // m_driverController.x().whileTrue(m_SwerveSubsystem.getRoutine().dynamic(Direction.kReverse));
     
  }
  /**
   * For cleanliness, register all named commands here
   */
  private void registerNamedCommands(){
      NamedCommands.registerCommand("outtake", m_CoralSubsystem.getOuttakeCommand().withTimeout(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return autoChooser.getSelected();
    return null;
  }
}
