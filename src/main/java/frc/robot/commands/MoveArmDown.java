package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class  MoveArmDown extends CommandBase {
    private Arm armSubsystem;

    public MoveArmDown(Arm arm) {

        armSubsystem = arm;
            addRequirements(armSubsystem);
        } 
     
    public void execute() {
        armSubsystem.moveDown();
        
    }    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.move(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}



