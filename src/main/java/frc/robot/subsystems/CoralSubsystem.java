package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
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
}
