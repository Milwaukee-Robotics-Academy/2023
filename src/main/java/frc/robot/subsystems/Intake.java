package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeController;
    public Intake(){
    intakeController = new CANSparkMax(Constants.Intake.motorCanID, MotorType.kBrushed);
       intakeController.restoreFactoryDefaults();
       intakeController.setIdleMode(IdleMode.kBrake);


    }

    public void intake()
    {
        intakeController.set(1.0);
    }
    
    public void outtake()
    {
     //   intakeController.set(-1);
    }

    public void run(double speed){
      //  intakeController.set(speed);
    }
    public void stop() {
      //  intakeController.set(0);
    }

    
    
}
