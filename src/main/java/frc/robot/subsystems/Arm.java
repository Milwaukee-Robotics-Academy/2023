package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {


    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private final CANSparkMax armMotor = new CANSparkMax(Constants.Arm.armMotorCanID, MotorType.kBrushless);
    private DigitalInput bottomlimitSwitch = new DigitalInput(0);

    public Arm() {
        armMotor.setInverted(true);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(40);
        armMotor.burnFlash();
    }

    public void moveUp() {
        armMotor.set(-0.5);
    }

    public void moveDown() {     
        if(!bottomlimitSwitch.get()){
            armMotor.set(0);
        } else {
            armMotor.set(0.5);
        }               

    }

    public void move(double speed){
        if(!bottomlimitSwitch.get() && speed>0){
            armMotor.set(0);
        } else {
            armMotor.set(speed);
        }     
    }
    public void stop() {
        armMotor.set(0);
    }


}
