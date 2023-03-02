package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Arm {

    private static final int deviceID = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private final CANSparkMax armMotor = new CANSparkMax(Constants.Arm.armMotorCanID, MotorType.kBrushless);

    public Arm() {
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
    public void Stop() {
        armMotor.set(0);
    }

}
