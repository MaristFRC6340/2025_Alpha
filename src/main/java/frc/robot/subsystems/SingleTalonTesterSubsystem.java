package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleTalonTesterSubsystem extends SubsystemBase{


    TalonFX motor;
    private Slot0Configs kSlot0Configs;

   
    public SingleTalonTesterSubsystem(){
        motor = new TalonFX(0);

        kSlot0Configs = new Slot0Configs().withKP(1);
        motor.getConfigurator().apply(new TalonFXConfiguration().withSlot0(kSlot0Configs));
    }

    public void setPower(double power){
        motor.set(power);
        SmartDashboard.putNumber("/Subsystem/Test/motor/powerSupplied", power);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("/Subsystem/Test/motor/position",motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("/Subsystem/Test/motor/velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("/Subsystem/Test/motor/angle", motor.getPosition().getValue().magnitude());
        SmartDashboard.putNumber("/Subsystem/Test/motor/voltage", motor.getMotorVoltage().getValueAsDouble());

    }

    public Command setPower(DoubleSupplier dubb){
        return this.runEnd(()->setPower(dubb.getAsDouble()),()->setPower(0));

    }


    

}