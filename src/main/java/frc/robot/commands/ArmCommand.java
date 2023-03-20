package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;


public class ArmCommand   extends CommandBase  {
  private Arm armSubsystem;
   private DoubleSupplier armStick;
   public ArmCommand (Arm armSubsystem, DoubleSupplier armStick) {
       this.armStick = armStick;
       this.armSubsystem = armSubsystem;
       addRequirements(armSubsystem);
   } 

public void execute()
{
    double armSpeed = armStick.getAsDouble();
    
    if (-0.05 <= armSpeed && armSpeed <= .05) {
        armSubsystem.stop();
        
    } else {
        armSubsystem.move(armSpeed * Constants.Arm.armMax);

    }


}
}