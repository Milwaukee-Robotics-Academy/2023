package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {


    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private final CANSparkMax armMotor = new CANSparkMax(Constants.Arm.armMotorCanID, MotorType.kBrushless);

    public Arm() {
        armMotor.setInverted(true);
        armMotor.setIdleMode(IdleMode.kBrake);
    }

    public void moveUp() {
        armMotor.set(0.5);
    }

    public void moveDown() {
        armMotor.set(-0.5);
    }

    public void move(double speed){
        armMotor.set(speed);
    }
    public void stop() {
        armMotor.set(0);
    }


}
