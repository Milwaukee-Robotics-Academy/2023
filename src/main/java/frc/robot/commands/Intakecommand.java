package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class Intakecommand extends CommandBase {
    private Intake intake;

    private DoubleSupplier intakeTrigger;
    private DoubleSupplier outakeTrigger;
    

    public Intakecommand (Intake intake, DoubleSupplier intakeTrigger, DoubleSupplier outakeTrigger ) {
        this.intake = intake;
        this.intakeTrigger = intakeTrigger;
        this.outakeTrigger = outakeTrigger;
        addRequirements(intake);


    }
    /* Double means the decimal number  */
    public void execute() {
        // if intake mode greater than .05 robot runs in intake mode
        double intakevalue = (intakeTrigger.getAsDouble());    
        // if outake mode greater than .05 robot runs in outake mode  
        double outakevalue = (outakeTrigger.getAsDouble());
        if (intakevalue > .05) {
            intake.intake(intakevalue * Constants.Intake.speedscale);
        } else if (outakevalue > .05){
            intake.outtake(outakevalue * Constants.Intake.speedscale);   
        } else {
            intake.stop();
        }

    }


    public Intakecommand () {

    }
}
