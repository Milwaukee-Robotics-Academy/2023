package frc.robot.subsystems;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public GenericEntry[] canCoderValues;
    public GenericEntry[] rotationValues;
    public GenericEntry[] velocityValues;
    
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
private PIDController driftCorrectionPID = new PIDController(.5, 0.00, 0.01,0.04);
private double previousXY = 0;
private double desiredHeading = 0;

    public Swerve() {

        zeroHeading(180);
        driftCorrectionPID.enableContinuousInput(-180, 180);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        canCoderValues = new GenericEntry[4];
        rotationValues = new GenericEntry[4];
        velocityValues = new GenericEntry[4];

        Shuffleboard.getTab("Swerve").add("navx",gyro).withSize(2,2).withPosition(0,0);

        for(SwerveModule mod : mSwerveMods){
            canCoderValues[mod.moduleNumber] = Shuffleboard.getTab("Swerve").add("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees()).withPosition((2+mod.moduleNumber),0).getEntry();
            rotationValues[mod.moduleNumber] = Shuffleboard.getTab("Swerve").add("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees()).withPosition((2+mod.moduleNumber),1).getEntry();
            velocityValues[mod.moduleNumber] = Shuffleboard.getTab("Swerve").add("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond).withPosition((2+mod.moduleNumber),2).getEntry();    
        }
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
       
       ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), 
            translation.getY(), 
            rotation, 
            getYaw()
        );

        /**
         * TODO: First get the Chassis speeds, run through drift correction then pass
         * see: https://www.chiefdelphi.com/t/mk4-drift/403628/28
         */
        SmartDashboard.putNumber("X", translation.getX());
        SmartDashboard.putNumber("Y", translation.getY());
        SmartDashboard.putNumber("rotation", rotation);

        double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
        if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || previousXY <= 0) desiredHeading = getPose().getRotation().getDegrees();
        else if(xy > 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(getPose().getRotation().getDegrees(), desiredHeading);
        previousXY = xy;
        
        SmartDashboard.putNumber("speeds.omegaRadiansPerSecond", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("desired Heading", desiredHeading);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? speeds
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public SwerveDriveKinematics getKinematics(){
        return Constants.Swerve.swerveKinematics;
    }
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void setDesiredHeading(double desired){
        desiredHeading = desired;
    }
    public void zeroHeading(){
        gyro.zeroYaw();
        desiredHeading = 0;
    }
    public void zeroHeading(double heading){
        gyro.zeroYaw();
        gyro.setAngleAdjustment(heading);  
        desiredHeading = heading;
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw()*-1);
    }
    public double getPitch() {
        return gyro.getPitch();
    }
    public double getRoll() {
        return gyro.getRoll();
    }
    
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        if (DriverStation.isDisabled()){
            for(SwerveModule mod : mSwerveMods){
                mod.limitCanCoderTraffic(false);
            }
            resetModulesToAbsolute();
        } else {
            for(SwerveModule mod : mSwerveMods){
                mod.limitCanCoderTraffic(true);
            }
        }

        for(SwerveModule mod : mSwerveMods){
            canCoderValues[mod.moduleNumber].setDouble(mod.getCanCoder().getDegrees());
            rotationValues[mod.moduleNumber].setDouble(mod.getPosition().angle.getDegrees());
            velocityValues[mod.moduleNumber].setDouble(mod.getState().speedMetersPerSecond);    
        }
        Logger.getInstance().recordOutput("Robot",(swerveOdometry.update(getYaw(), getModulePositions())));  
               // SmartDashboard.putNumber("Yaw",getYaw().getDegrees());

    }
}