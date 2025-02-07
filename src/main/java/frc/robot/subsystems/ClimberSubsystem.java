package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{

    TalonFX climberMotor;
    Slot0Configs slot0config;

    public ClimberSubsystem() {

        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberId);
        slot0config = new Slot0Configs();
        slot0config.kP=1;
        climberMotor.getConfigurator().apply(slot0config);

    }


    public Command setPower(DoubleSupplier pow){
        return this.runEnd(()->{
            climberMotor.set(pow.getAsDouble());
        },()->{
           //climberMotor.setControl(new PositionVoltage(0).withSlot(0).withPosition(climberMotor.getPosition().getValueAsDouble))
           //some hold position code here
        });
    }

    public void periodic() {
        
    }
}
