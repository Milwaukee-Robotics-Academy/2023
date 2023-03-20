package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class CenterReverse extends SequentialCommandGroup {
    public CenterReverse(Swerve swerve, Arm arm, Intake intake) {
        addCommands(new SequentialCommandGroup(
            new InstantCommand(() -> arm.move(.5)),
            new WaitCommand(2),
            new InstantCommand(() -> arm.move(0)),
            new InstantCommand(()-> intake.outtake(.5)),
            new WaitCommand(2),
            new InstantCommand(()-> intake.outtake(0))));
            // new Start2Balance(swerve),
            // new AutoBalance(swerve)));
            // 
        }
}










