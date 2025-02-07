package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HuggerSubsystem extends SubsystemBase{

    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private TalonFX pivotMotor;

    private PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0); //Position in rotations

    public HuggerSubsystem() {

        //Pivot Motor
        pivotMotor = new TalonFX(Constants.HuggerConstants.kPivotID);
        Slot0Configs kPivotConfig = new Slot0Configs();
        kPivotConfig.kP=Constants.HuggerConstants.kPPivot;
        kPivotConfig.kG=Constants.HuggerConstants.kGPivot;
        pivotMotor.getConfigurator().apply(kPivotConfig);
        pivotMotor.setControl(m_PositionVoltage);
        //Left Motor
        leftMotor = new SparkMax(Constants.HuggerConstants.kLeftID, MotorType.kBrushless);
        SparkMaxConfig kLeftConfig= new SparkMaxConfig();
        kLeftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        leftMotor.configure(kLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        leftMotor = new SparkMax(Constants.HuggerConstants.kLeftID, MotorType.kBrushless);
        SparkMaxConfig kRightConfig= new SparkMaxConfig();
        kRightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        rightMotor.configure(kRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * 
     * @param position the position in rotations
     * @return command to set position
     */
    public Command setPosition(double position) {
        return this.runOnce(() -> {
            pivotMotor.setControl(m_PositionVoltage.withPosition(position));
        });
    }

    public Command getSetSpeedCommand(double speed) {
        return this.startEnd(() -> {
            setSpeed(speed);
        }, () -> {
            stop();
        });
    }

    public void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }
    
    public void stop() {
        setSpeed(0);
    }
}
