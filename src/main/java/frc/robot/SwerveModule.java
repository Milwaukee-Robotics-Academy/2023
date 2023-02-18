package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.COTSNEOSwerveConstants;
import frc.lib.util.NEOModuleState;
import frc.lib.util.SwerveModuleConstants;

import org.opencv.imgproc.Moments;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANCoder angleEncoder;
   // private final SparkMaxPIDController rotationController;
    private PIDController testRotationController;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        // rotationController = mAngleMotor.getPIDController();
        // rotationController.setP(0.5);
        // rotationController.setI(0.0);
        // rotationController.setD(0.0);
        testRotationController = new PIDController(0.5,0.0,0.0);
        testRotationController.enableContinuousInput(-Math.PI, Math.PI);


        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = NEOModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
        //     double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        //     mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        // TODO: configure for closed loop
        double angularVelolictySetpoint = desiredState.speedMetersPerSecond /
        (COTSNEOSwerveConstants.SDSMK4i(Constants.Swerve.driveGearRatio).wheelDiameter /2.0);
        mDriveMotor.set(desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed);
    }
    }
    

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        //mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        
         mAngleMotor.set(testRotationController.calculate(getAngle().getRadians(),angle.getRadians()));
       // rotationController.set(Conversions.(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){   
        double unsignedAngle = angleEncoder.getPosition() % (2 * Math.PI);

        if (unsignedAngle < 0)
          unsignedAngle += 2 * Math.PI;
    
        return new Rotation2d(unsignedAngle);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
      //  double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
     // TODO: Need to validate this. This is a guess so far...
      mAngleMotor.getEncoder().setPosition((getAngle().getRadians()-angleOffset.getRadians()));
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        // mAngleMotor.restoreFactoryDefaults();
        // mAngleMotor.configAllSettings(Robot.neoConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setInverted(COTSNEOSwerveConstants.SDSMK4i(Constants.Swerve.driveGearRatio).angleMotorInvert);
        //switch to a constant
        mAngleMotor.setIdleMode(IdleMode.kCoast);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        // mDriveMotor.configFactoryDefault();
        // mDriveMotor.configAllSettings(Robot.neoConfigs.swerveDriveFXConfig);
        // mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        // mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        // mDriveMotor.setSelectedSensorPosition(0);
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setInverted(COTSNEOSwerveConstants.SDSMK4i(Constants.Swerve.driveGearRatio).driveMotorInvert);
        mDriveMotor.setIdleMode(IdleMode.kBrake);
        mDriveMotor.getEncoder().setPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            //Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            mDriveMotor.getEncoder().getVelocity() * (COTSNEOSwerveConstants.SDSMK4i(Constants.Swerve.driveGearRatio).wheelDiameter /2.0),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            // Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
           mDriveMotor.getEncoder().getPosition() * (COTSNEOSwerveConstants.SDSMK4i(Constants.Swerve.driveGearRatio).wheelDiameter /2.0),
            getAngle()
        );
    }
}