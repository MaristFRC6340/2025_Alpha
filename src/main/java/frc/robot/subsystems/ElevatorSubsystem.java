package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@Logged

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

    //Clamps the position value
    public void setPosition(double position) {
        leftMotor.setControl(m_MMEV.withPosition(Math.max(Constants.ElevatorConstants.kMin, Math.min(Constants.ElevatorConstants.kMax, position)) ));
    }
    

    public Command getSetPositionCommand(double position) {
        return this.runOnce(() -> {
            setPosition(position);
        });
    }

    public Command goHome() {
        return getSetPositionCommand(0);
    }
}
