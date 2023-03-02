package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
//    private Spark intakeController;
    private PWMSparkMax intakeController;

    public IntakeSubsystem()
    {
//        intakeController = new Spark(99);
        intakeController = new PWMSparkMax(0);
    }

    public void intake()
    {
        intakeController.set(1);
    }
    
    public void outtake()
    {
        intakeController.set(-1);
    }

    public void stop() {
        intakeController.set(0);
    }

    
}
 