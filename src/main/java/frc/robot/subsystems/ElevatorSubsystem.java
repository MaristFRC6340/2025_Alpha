package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX leftMotor;
    TalonFX rightMotor;

    private final MotionMagicExpoVoltage m_MMEV= new MotionMagicExpoVoltage(0); //Position in rotations
    private final Follower m_Follower = new Follower(leftMotor.getDeviceID(), true);

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(Constants.ElevatorConstants.kLeftID);
        leftMotor.getConfigurator().apply(Constants.ElevatorConstants.kLeftConfig);

        rightMotor = new TalonFX(Constants.ElevatorConstants.kRightID);
        rightMotor.getConfigurator().apply(Constants.ElevatorConstants.kRightConfig);

        leftMotor.setControl(m_MMEV);
        rightMotor.setControl(m_Follower);
    }
    

    public Command setPosition(double position) {
        return this.runOnce(() -> {
            leftMotor.setControl(m_MMEV.withPosition(position));
        });
    }

    public Command goHome() {
        return setPosition(0);
    }
}
