package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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
        return this.startEnd(() -> {
            onOuttake(CoralConstants.outtakeMotorSpeed);
        },
        
        () -> {
            stop();
        });
    }

    public void onOuttake(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
    }

    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    @Override
    public void periodic() {

    }

}
