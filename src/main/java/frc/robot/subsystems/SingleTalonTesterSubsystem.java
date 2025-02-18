package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleTalonTesterSubsystem extends SubsystemBase{


    TalonFX motor;
    CANcoder c;
    private Slot0Configs kSlot0Configs;
    AHRS m_Gyro;

   
    public SingleTalonTesterSubsystem(){
        motor = new TalonFX(10);
        c = new CANcoder(12);
        m_Gyro = new AHRS(NavXComType.kMXP_SPI);
       

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
        SmartDashboard.putNumber("/Subsystem/Test/cancoder/position",c.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("/Subsystem/Test/gyro/ori",m_Gyro.getAngle());

    }

    public Command setPower(DoubleSupplier dubb){
        return this.runEnd(()->setPower(dubb.getAsDouble()),()->setPower(0));

    }


    

}