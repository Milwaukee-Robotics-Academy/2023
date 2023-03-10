package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class  IntakeOut extends CommandBase {
    private Intake m_intake;

    public IntakeOut(Intake intake) {

        m_intake = intake;
            addRequirements(m_intake);
        } 
     
    public void execute() {
        m_intake.outtake(0.5);
        
    }  
    public void end() {  
       m_intake.outtake(0.0);
    }
}



