package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged

public class CoralSubsystem extends SubsystemBase{
    
    private SparkMax topMotor;
    private SparkMax bottomMotor;

    public CoralSubsystem() {
        topMotor = new SparkMax(Constants.CoralConstants.kTopID, MotorType.kBrushless);
        topMotor.configure(Constants.CoralConstants.kTopConfig, null, null);

        bottomMotor = new SparkMax(Constants.CoralConstants.kBottomID, MotorType.kBrushless);
        bottomMotor.configure(Constants.CoralConstants.kBottomConfig, null, null);
    }
// goal: on button press, get both motors to spin same direction at an equal speed
// when putting the coral on top, set a different speed for the motors

    public Command getOuttakeCommand() {
        return getSetSpeedCommand(CoralConstants.kOuttakeMotorSpeed);
    }

    public Command getSetSpeedCommand(double speed) {
        return this.startEnd(() -> {
            setSpeed(speed);
        }, () -> {
            stop();
        });
    }

    public Command getShadowTechniqueCommand(double speed) {
        return this.startEnd(() -> {
            shadowTechnique(speed);
        }, () -> {
            stop();
        });
    }

    public void setSpeed(double speed) {
        topMotor.set(-speed);
        bottomMotor.set(speed);
    }

    public void shadowTechnique(double speed) {
        topMotor.set(-speed);
        bottomMotor.set(-speed);
    }

    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    @Override
    public void periodic() {
            SmartDashboard.putNumber("Subsystem/Coral/Top/position", topMotor.getAlternateEncoder().getPosition());
            SmartDashboard.putNumber("Subsystem/Coral/Top/velocity", topMotor.getAlternateEncoder().getVelocity());
            SmartDashboard.putNumber("Subsystem/Coral/Bottom/position", bottomMotor .getAlternateEncoder().getPosition());
            SmartDashboard.putNumber("Subsystem/Coral/Bottom/velocity", bottomMotor.getAlternateEncoder().getVelocity());
            SmartDashboard.putData("Subsystem/Coral/currentCommand", this.getCurrentCommand());
            SmartDashboard.putData("Subsystem/Coral/defaultCommand", this.getDefaultCommand());
    }         


}
