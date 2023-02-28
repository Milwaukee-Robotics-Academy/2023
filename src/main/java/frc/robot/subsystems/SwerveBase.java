package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class SwerveBase extends SubsystemBase {

  /**
   * Subsystem that controls the drivetrain of the robot
   * Handles all the odometry and base movement for the chassis
   */

  /**
   * absolute encoder offsets for the wheels
   * 180 degrees added to offset values to invert one side of the robot so that it
   * doesn't spin in place
   */
  private static final double frontLeftAngleOffset = Units.degreesToRadians(110.65);
  private static final double frontRightAngleOffset = Units.degreesToRadians(302.78);
  private static final double rearLeftAngleOffset = Units.degreesToRadians(223.33);
  private static final double rearRightAngleOffset = Units.degreesToRadians(257.78);

  /**
   * SwerveModule objects
   * Parameters:
   * drive motor can ID
   * rotation motor can ID
   * external CANCoder can ID
   * measured CANCoder offset
   */

  private final SwerveModule frontLeft = new SwerveModule(
      Swerve.frontLeftDriveMotorId,
      Swerve.frontLeftRotationMotorId,
      Swerve.frontLeftRotationEncoderId,
      frontLeftAngleOffset);

  private final SwerveModule frontRight = new SwerveModule(
      Swerve.frontRightDriveMotorId,
      Swerve.frontRightRotationMotorId,
      Swerve.frontRightRotationEncoderId,
      frontRightAngleOffset);

  public SwerveModule getFrontRight() {
    return frontRight;
  }

  private final SwerveModule rearLeft = new SwerveModule(
      Swerve.rearLeftDriveMotorId,
      Swerve.rearLeftRotationMotorId,
      Swerve.rearLeftRotationEncoderId,
      rearLeftAngleOffset);

  private final SwerveModule rearRight = new SwerveModule(
      Swerve.rearRightDriveMotorId,
      Swerve.rearRightRotationMotorId,
      Swerve.rearRightRotationEncoderId,
      rearRightAngleOffset);

  private final AHRS navX;

  /**
   * odometry for the robot, measured in meters for linear motion and radians for
   * rotational motion
   * Takes in kinematics and robot angle for parameters
   */
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Swerve.kinematics, new Rotation2d(),
      getModulePositions());
  private PIDController driftCorrectionPID = new PIDController(0.15, 0.00, 0.0,0.04);
  private double desiredHeading;
  private double previousXY;

  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  public SwerveDriveKinematics getKinematics() {
    return Swerve.kinematics;
  }
  public SwerveBase() {
    navX = new AHRS(SPI.Port.kMXP);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    // odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
    Timer.delay(2);
    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    rearLeft.initRotationOffset();
    rearRight.initRotationOffset();

    // reset the measured distance driven for each module
    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

    rearRight.getDriveMotor().setInverted(true);
    rearLeft.getDriveMotor().setInverted(true);
    frontRight.getDriveMotor().setInverted(true);
    frontLeft.getDriveMotor().setInverted(true);

    rearRight.getRotationMotor().setInverted(true);
    rearLeft.getRotationMotor().setInverted(true);
    frontRight.getRotationMotor().setInverted(true);
    frontLeft.getRotationMotor().setInverted(true);
    driftCorrectionPID.enableContinuousInput(-180, 180);

  }

  @Override
  public void periodic() {

    // update the odometry every 20ms
    odometry.update(getHeading(), getModulePositions());

    SmartDashboard.putString("Robot pose",
        getPose().toString());
    SmartDashboard.putNumber("navX Heading",
        getHeading().getDegrees());

    SmartDashboard.putNumber("roll",
        navX.getRoll());

    SmartDashboard.putNumber("pitch",
        navX.getPitch());
    SmartDashboard.putNumber("LeftFrontEnc", frontLeft.getIntegratedAngle().getDegrees());
    SmartDashboard.putNumber("RightFrontEnc",frontRight.getIntegratedAngle().getDegrees());
    SmartDashboard.putNumber("LeftRearEnc",rearLeft.getIntegratedAngle().getDegrees());
    SmartDashboard.putNumber("RightRearEnc",rearRight.getIntegratedAngle().getDegrees());

    SmartDashboard.putNumber("LeftFront drive enc", frontLeft.getDriveDistanceRadians());
    SmartDashboard.putNumber("RightFront drive enc",frontRight.getDriveDistanceRadians());
    SmartDashboard.putNumber("LeftRear drive Enc",rearLeft.getDriveDistanceRadians());
    SmartDashboard.putNumber("RightRear drive Enc",rearRight.getDriveDistanceRadians());
    SmartDashboard.putData(navX);
  }

  /**
   * method for driving the robot
   * Parameters:
   * forward linear value
   * sideways linear value
   * rotation value
   * if the control is field relative or robot relative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      forward, 
      strafe, 
      rotation, 
      getHeading()
  );
  SmartDashboard.putNumber("CommandX",forward);
  SmartDashboard.putNumber("CommandY",strafe);
     double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
     if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || previousXY <= 0) desiredHeading = getPose().getRotation().getDegrees();
     else if(xy > 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(getPose().getRotation().getDegrees(), desiredHeading);
     previousXY = xy;

     speeds = isFieldRelative
        ? speeds
        : new ChassisSpeeds(forward, strafe, rotation);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = Swerve.kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states);

  }

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative, boolean isAutoBalancing) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      forward, 
      strafe, 
      rotation, 
      getHeading()
  );
     double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);
    if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || previousXY <= 0) desiredHeading = getPose().getRotation().getDegrees();
     else if(xy > 0) speeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(getPose().getRotation().getDegrees(), desiredHeading);
     previousXY = xy;

     speeds = isFieldRelative
        ? speeds
        : new ChassisSpeeds(forward, strafe, rotation);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = Swerve.kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states, isAutoBalancing);

  }

  public void setDesiredHeading(double heading){
    desiredHeading = heading;
  }
    


  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Swerve.maxSpeed);
    frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
    frontRight.setDesiredStateClosedLoop(moduleStates[1]);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
    rearRight.setDesiredStateClosedLoop(moduleStates[3]);

  }

  /**
   * Method to set the desired state for each swerve module
   * Uses PID and feedforward control to control the linear and rotational values
   * for the modules
   */
  public void setModuleStates(SwerveModuleState[] moduleStates, boolean isAutoBalancing) {
    // make sure the wheels don't try to spin faster than the maximum speed possible
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Swerve.maxSpeed);
    frontLeft.setDesiredStateClosedLoop(moduleStates[0], isAutoBalancing);
    frontRight.setDesiredStateClosedLoop(moduleStates[1], isAutoBalancing);
    rearLeft.setDesiredStateClosedLoop(moduleStates[2], isAutoBalancing);
    rearRight.setDesiredStateClosedLoop(moduleStates[3], isAutoBalancing);

  }

  // returns an array of SwerveModuleState
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = {
        new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return states;

  }

  // returns an array of SwerveModulePositions
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeft.getCurrentDistanceMetersPerSecond(), frontLeft.getIntegratedAngle()),
        new SwerveModulePosition(frontRight.getCurrentDistanceMetersPerSecond(), frontRight.getIntegratedAngle()),
        new SwerveModulePosition(rearLeft.getCurrentDistanceMetersPerSecond(), rearLeft.getIntegratedAngle()),
        new SwerveModulePosition(rearRight.getCurrentDistanceMetersPerSecond(), rearRight.getIntegratedAngle())

    };

    return positions;

  }

  /**
   * Return the current position of the robot on field
   * Based on drive encoder and gyro reading
   */
  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  // reset the current pose to a desired pose
  public void resetOdometry(Pose2d pose) {

    odometry.resetPosition(getHeading(), getModulePositions(), pose);

  }

  // reset the measured distance driven for each module
  public void resetDriveDistances() {

    frontLeft.resetDistance();
    frontRight.resetDistance();
    rearLeft.resetDistance();
    rearRight.resetDistance();

  }

  // get the current heading of the robot based on the gyro
  public Rotation2d getHeading() {

    return Rotation2d.fromDegrees(navX.getYaw()*-1);

  }

  public AHRS getNavX() {
    return navX;
  }

  public void zeroHeading() {
    navX.reset();
    desiredHeading = 0;
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  public SequentialCommandGroup followPathCmd(String pathName) {

    PathPlannerTrajectory trajectory = getPathPlannerTrajectory(pathName);

    Command ppCommand = getPathPlannerCommand(trajectory);

    return new SequentialCommandGroup(

        new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose())),
        ppCommand,
        new InstantCommand(() -> this.stopModules()));
  }

  public PathPlannerTrajectory getPathPlannerTrajectory(String pathName) {

    PathConstraints constraints = PathPlanner.getConstraintsFromPath(pathName);
    PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathName, constraints, false);
    return ppTrajectory;

  }

  public Command getPathPlannerCommand(PathPlannerTrajectory trajectory) {

    PIDController xController = new PIDController(3, 0, 0);
    PIDController yController = new PIDController(3, 0, 0);
    PIDController thetaController = new PIDController(
        1, 0.0, 0.0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        Swerve.kinematics,
        xController,
        yController,
        thetaController,
        this::setModuleStates,
        this);

    return command;
  }

}