package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class CenterReverse extends SequentialCommandGroup {
    public CenterReverse(Swerve swerve, Arm arm, Intake intake) {
        addCommands(new SequentialCommandGroup(
            new InstantCommand(() -> arm.move(.3)).withTimeout(0.5),
            new InstantCommand(()-> intake.outtake(.5)).withTimeout(1),
            new Start2Balance(swerve)));
            }
}










