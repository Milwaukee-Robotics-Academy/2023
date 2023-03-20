package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier m_speedReduction;
    private BooleanSupplier robotCentricSup;
    private PIDController driftCorrectionPID = new PIDController(0.3, 0.00, 0.01, 0.04);
    private double previousXY = 0;
    private double desiredHeading = 0;
    private DoubleSupplier commandedHeading;

    public Drive(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup,
            DoubleSupplier commandedHeading, DoubleSupplier speedReduction, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.m_speedReduction = speedReduction;
        this.robotCentricSup = robotCentricSup;
        this.commandedHeading = commandedHeading;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        /* Sup stands for suppliers, Val stands value, */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
 
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVal * Constants.Swerve.maxSpeed * m_speedReduction.getAsDouble(),
                strafeVal * Constants.Swerve.maxSpeed * m_speedReduction.getAsDouble(),
                rotationVal * Constants.Swerve.maxAngularVelocity * m_speedReduction.getAsDouble(),
                s_Swerve.getYaw());

        // if d-pad desired heading is commanded, then rotate to that
        if (Math.abs(commandedHeading.getAsDouble()) < 181) {
            desiredHeading = commandedHeading.getAsDouble();
        } else if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0) {
            // we are turning, so set the desired and the current the same
            desiredHeading = s_Swerve.getPose().getRotation().getDegrees();
        }
        if ((Math.abs(translationVal) + Math.abs(strafeVal)) > 0) {
         //  we are moving x or y, so add drift correction
            speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(s_Swerve.getPose().getRotation().getDegrees(),
                    desiredHeading);
        } else {
            desiredHeading = s_Swerve.getPose().getRotation().getDegrees();
        }
        SmartDashboard.putNumber("desired Heading", desiredHeading);
        /* Drive */
        s_Swerve.drive(speeds);

    }


    @Override
    public void end(boolean interupted) {
        s_Swerve.drive(new ChassisSpeeds());
    }
}