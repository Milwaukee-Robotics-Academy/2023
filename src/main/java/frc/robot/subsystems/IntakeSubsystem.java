package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private final CANSparkMax intakeController = new CANSparkMax(Constants.Intake.motorCanID, MotorType.kBrushed);

    public IntakeSubsystem(){
    }

    public void intake()
    {
        intakeController.set(1);
    }
    
    public void outtake()
    {
        intakeController.set(-1);
    }

    public void run(double speed){
        intakeController.set(speed);
    }
    public void stop() {
        intakeController.set(0);
    }

    
}
 