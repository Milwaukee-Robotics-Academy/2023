package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class  IntakeIn extends CommandBase {
    private Intake m_intake;

    public IntakeIn(Intake intake) {

        m_intake = intake;
            addRequirements(m_intake);
        } 
     
    public void execute() {
        m_intake.intake(0.5);
        
    }  
    @Override
    public void end(boolean interrupted) {
        m_intake.intake(0.0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}



