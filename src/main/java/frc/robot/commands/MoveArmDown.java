package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
    public void end() {  
        armSubsystem.move(0);
    }
}



