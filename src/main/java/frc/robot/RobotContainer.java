// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HuggerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HuggerSubsystem;
import frc.robot.subsystems.SingleTalonTesterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final   HuggerSubsystem m_HuggerSubsystem = new HuggerSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  //private final SingleTalonTesterSubsystem motor = new SingleTalonTesterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final CommandXboxController m_actuatorController = new CommandXboxController(OperatorConstants.kActuatorControllerPort);

  private final CommandPS5Controller m_actuatorCommandPS5Controller = new CommandPS5Controller(OperatorConstants.kActuatorControllerPort);
  //Buttons!
  // private Trigger actuatorA = m_actuatorController.a();
  // private Trigger actuatorB = m_actuatorController.b();
  // private Trigger actuatorX = m_actuatorController.x();
  // private Trigger actuatorY = m_actuatorController.y();
  // private Trigger actuatorL = m_actuatorController.leftBumper();
  // private Trigger actuatorR = m_actuatorController.rightBumper();
  // private Trigger actuatorStart = m_actuatorController.start();
  // private Trigger actuatorBack = m_actuatorController.back();
  // private Trigger actuatorLTrigger = m_actuatorController.leftTrigger(.05);
  // private Trigger actuatorRTrigger = m_actuatorController.rightTrigger(.05);
  // private Trigger actuatorLStick = new Trigger(() -> Math.abs(m_actuatorController.getLeftY()) > .05);
  // private Trigger actuatorRStick = new Trigger(() -> Math.abs(m_actuatorController.getRightY()) > .05);
  // private Trigger actuatorDpadUp = m_actuatorController.povUp();
  // private Trigger actuatorDpadRight = m_actuatorController.povRight();
  // private Trigger actuatorDpadDown = m_actuatorController.povDown();
  // private Trigger actuatorDpadLeft = m_actuatorController.povLeft();

  //Play Station Controller
  private Trigger actuatorA = m_actuatorCommandPS5Controller.cross();
  private Trigger actuatorB = m_actuatorCommandPS5Controller.circle();
  private Trigger actuatorX = m_actuatorCommandPS5Controller.square();
  private Trigger actuatorY = m_actuatorCommandPS5Controller.triangle();
  private Trigger actuatorL = m_actuatorCommandPS5Controller.L1();
  private Trigger actuatorR = m_actuatorCommandPS5Controller.R1();
  private Trigger actuatorStart = m_actuatorCommandPS5Controller.options();
  private Trigger actuatorLTrigger = m_actuatorCommandPS5Controller.axisGreaterThan(3,.05);
  private Trigger actuatorRTrigger = m_actuatorCommandPS5Controller.axisGreaterThan(4,.05);
  private Trigger actuatorLStick = new Trigger(() -> Math.abs(m_actuatorCommandPS5Controller.getLeftY()) > .05);
  private Trigger actuatorRStick = new Trigger(() -> Math.abs(m_actuatorCommandPS5Controller.getRightY()) > .05);
  private Trigger actuatorDpadUp = m_actuatorCommandPS5Controller.povUp();
  private Trigger actuatorDpadRight = m_actuatorCommandPS5Controller.povRight();
  private Trigger actuatorDpadDown = m_actuatorCommandPS5Controller.povDown();
  private Trigger actuatorDpadLeft = m_actuatorCommandPS5Controller.povLeft();


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
  

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),
  () -> m_driverController.getLeftY() * 1,
  () -> m_driverController.getLeftX() * 1)
  .withControllerRotationAxis(() -> m_driverController.getRightX()*-1)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);
  SwerveInputStream driveAngularSlow = SwerveInputStream.of(m_SwerveSubsystem.getSwerveDrive(),
  () -> m_driverController.getLeftY() * .25,
  () -> m_driverController.getLeftX() * .25)
  .withControllerRotationAxis(() -> m_driverController.getRightX()*-.25)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);
  

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
    // m_elevator.setDefaultCommand(m_elevator.setPower(()->m_driverController.getLeftTriggerAxis()*.125));
    // driverLTrigger.whileTrue(m_elevator.setPower(()->m_driverController.getLeftTriggerAxis()*.2));
    // driverRTrigger.whileTrue(m_elevator.setPower(()->m_driverController.getRightTriggerAxis()*-.2));

    // driverY.onTrue(m_elevator.getSetPositionCommand(3));
    // driverX.onTrue(m_elevator.getSetPositionCommand(1.5));
    // driverA.onTrue(m_elevator.getSetPositionCommand(.1));
    // m_SwerveSubsystem.setDefaultCommand(m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity));
    // driverStart.onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    // actuatorL.whileTrue(m_CoralSubsystem.getSetSpeedCommand(.9));
    // actuatorR.whileTrue(m_CoralSubsystem.getShadowTechniqueCommand(.5));
    // actuatorA.whileTrue(m_HuggerSubsystem.getSetSpeedCommand(() -> .5));
    // actuatorY.whileTrue(m_HuggerSubsystem.getSetSpeedCommand(() -> -.5));
    
    
      
      //DRIVER CONTROLS
        m_SwerveSubsystem.setDefaultCommand(m_SwerveSubsystem.driveFieldOriented(driveAngularVelocity));
        driverRTrigger.whileTrue(m_SwerveSubsystem.driveFieldOriented(driveAngularSlow));
        driverLTrigger.whileTrue(m_SwerveSubsystem.driveRobotCentric(driveAngularVelocity));
       
       
        driverL.whileTrue(m_SwerveSubsystem.getDriveToClosestReefPoseCommand(true));
        driverR.whileTrue(m_SwerveSubsystem.getDriveToClosestReefPoseCommand(false));
        //driverR.onTrue(new RunCommand(()->m_SwerveSubsystem.drive(new Translation2d(0,1),Math.toRadians(0),false)));


        //DPAD Drive To Commands
        // driverDpadUp.whileTrue(m_SwerveSubsystem.driveToPose(Constants.FieldPositions.BLUE_CLIMB_AREA));
        // driverDpadRight.whileTrue(m_SwerveSubsystem.driveToPose(Constants.FieldPositions.BLUE_PROCESSOR));
        // driverDpadLeft.whileTrue(m_SwerveSubsystem.driveToPose(Constants.FieldPositions.BLUE_LEFT_CORAL_STATION_PICKUP));

        driverA.onTrue(new InstantCommand(()->m_SwerveSubsystem.zeroGyro()));
       

      //ACTUATOR CONTROLLER

      actuatorA.whileTrue(new InstantCommand(()->m_HuggerSubsystem.setPosition(Constants.HuggerConstants.intakeAlgaePosition)).andThen(()->m_elevator.setPosition(ElevatorConstants.lowerAlgaeHeight)));
      actuatorX.whileTrue(new InstantCommand(()->m_HuggerSubsystem.setPosition(Constants.HuggerConstants.outtakeAlgaeProcessor)).andThen(()->m_elevator.setPosition(ElevatorConstants.processorAlgaeHight)));
      actuatorY.whileTrue(new InstantCommand(()->m_HuggerSubsystem.setPosition(Constants.HuggerConstants.intakeAlgaePosition)).andThen(()->m_elevator.setPosition(ElevatorConstants.upperAlgaeHeight)));

      actuatorB.whileTrue(m_CoralSubsystem.getL4OuttakeCommand());
      actuatorDpadUp.onTrue(new InstantCommand(()->m_elevator.increaseCoralState()));
      actuatorDpadDown.onTrue(new InstantCommand(()->m_elevator.decreseCoralState()));
      actuatorDpadLeft.onTrue(new InstantCommand(()->m_elevator.setCoralIntake()));
      
      actuatorLStick.whileTrue(m_elevator.setPower(() -> -1*m_actuatorCommandPS5Controller.getLeftY()*.25));
       //actuatorLStick.whileTrue(m_ClimberSubsystem.setPower(() -> m_actuatorController.getLeftY()*.25));
      
     actuatorRStick.whileTrue(m_HuggerSubsystem.getSetPivotPower(() -> -1*m_actuatorCommandPS5Controller.getRightY()*.1));
      //actuatorY.whileTrue(new InstantCommand(()->m_HuggerSubsystem.setPosition(-8)));
      actuatorL.whileTrue(m_CoralSubsystem.getSetSpeedCommand(1));
      actuatorR.whileTrue(m_CoralSubsystem.getShadowTechniqueCommand(.5));
      
      actuatorRTrigger.whileTrue(m_HuggerSubsystem.getSetSpeedCommand(()->.8));
      actuatorLTrigger.whileTrue(m_HuggerSubsystem.getSetSpeedCommand(()->-.8));

      actuatorStart.onTrue(new InstantCommand(()->m_HuggerSubsystem.setPosition(HuggerConstants.straightUp)));

      
  }
  /**
   * For cleanliness, register all named commands here
   */
  private void registerNamedCommands(){
      //NamedCommands.registerCommand("outtake", m_CoralSubsystem.getOuttakeCommand().withTimeout(1));
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
