package frc.robot.autos;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveBase;


public class AutoBalance extends CommandBase {    
    private SwerveBase s_Swerve;

    public AutoBalance(SwerveBase s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double driveSpeed;

        if(s_Swerve.getPitch() > 5){
                driveSpeed = -.13;
            } else if(s_Swerve.getPitch() < -5){
            driveSpeed = .13;
            } else {driveSpeed = 0;}

        /* Drive */
        s_Swerve.drive(
            driveSpeed * Constants.Swerve.maxSpeed, 
            0, 
            0, 
            true
        );
    }

    private static int getPitch() {
        return 0;
    }
}