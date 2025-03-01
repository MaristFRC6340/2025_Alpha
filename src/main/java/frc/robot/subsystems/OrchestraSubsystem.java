package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OrchestraSubsystem extends SubsystemBase{

    Orchestra m_orchestra;
    public OrchestraSubsystem(TalonFX... talons){
         m_orchestra = new Orchestra();

        // Add a single device to the orchestra
        for(TalonFX talon: talons ){
            m_orchestra.addInstrument(talon);
        }

        // Attempt to load the chrp
        var status = m_orchestra.loadMusic("sr.chrp");

        if (!status.isOK()) {
        // log error
        }
    }

    public void play(){
        m_orchestra.play();

    }

    

}