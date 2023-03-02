package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        double intakevalue = (intakeTrigger.getAsDouble());      
        double outakevalue = (outakeTrigger.getAsDouble());
        if (intakevalue > 0.5) {
            intake.intake();
        } else if (outakevalue > 0.5){
            intake.outtake();   
        } else {
            intake.stop();
        }

    }


    public Intakecommand () {

    }
}
