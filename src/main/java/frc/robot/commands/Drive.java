package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Drive extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier m_speedReduction;
    private BooleanSupplier robotCentricSup;
    private PIDController driftCorrectionPID = new PIDController(0.5, 0.00, 0.01,0.04);
    private double previousXY = 0;
    private DoubleSupplier desiredHeading = 0;
    
    public Drive(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup,DoubleSupplier commandedHeading, DoubleSupplier speedReduction, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.m_speedReduction = speedReduction;
        this.robotCentricSup = robotCentricSup;
        this.desiredHeading = commandedHeading;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        /*Sup stands for suppliers, Val stands value, */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translationVal, 
            strafeVal, 
            rotationVal, 
            s_Swerve.getYaw()
        );
        
        if(Math.abs(desiredHeading)<181){
            if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || previousXY <= 0) desiredHeading = s_Swerve.getPose().getRotation().getDegrees();
            else if((Math.abs(translationVal)+Math.abs(strafeVal))> 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(getPose().getRotation().getDegrees(), desiredHeading);
        }
        /* Drive */
        s_Swerve.drive(speeds);
    }

    @Override
    public void end(Boolean interupted){
        s_Swerve.drive(new ChassisSpeeds());
    }
}