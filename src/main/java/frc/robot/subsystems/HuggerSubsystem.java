package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HuggerSubsystem extends SubsystemBase{

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private TalonFX pivotMotor;

    private PositionVoltage m_PositionVoltage = new PositionVoltage(0);

    public HuggerSubsystem() {
        
    }
}
