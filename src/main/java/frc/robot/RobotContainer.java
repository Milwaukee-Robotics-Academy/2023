package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int intakeAxis = XboxController.Axis.kLeftTrigger.value;
    private final int outtakeAxis = XboxController.Axis.kRightTrigger.value;
    private final JoystickButton slow;
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    public final POVButton driverUP;
    public final POVButton driverDOWN;
    public final POVButton driverLEFT;
    public final POVButton driverRIGHT;
    private POVButton operatorUP;
    private POVButton operatorDOWN;
    private POVButton operatorLEFT;
    private POVButton operatorRIGHT;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final IntakeSubsystem intake = new IntakeSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));
        intake.setDefaultCommand(
                new Intakecommand(intake, () -> driver.getRawAxis(intakeAxis), () -> driver.getRawAxis(outtakeAxis)));
        slow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
        driverUP = new POVButton(driver, 0);
        driverRIGHT = new POVButton(driver, 90);
        driverDOWN = new POVButton(driver, 180);
        driverLEFT = new POVButton(driver, 270);
        operatorUP = new POVButton(operator, 0);
        operatorRIGHT = new POVButton(operator, 90);
        operatorDOWN = new POVButton(operator, 180);
        operatorLEFT = new POVButton(operator, 270);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Return how much the speed should be reduced.
     * 
     * @return speed reduced percent
     */
    public double speedReduction() {
        if (slow.getAsBoolean()) {
            SmartDashboard.putBoolean("SpeedReduced", true);
            return 0.4;
        } else {
            SmartDashboard.putBoolean("SpeedReduced", false);
            return 1.0;
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driverUP.onTrue(new TurnToAngleCmd(
                s_Swerve,
                0.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        driverDOWN.onTrue(new TurnToAngleCmd(
                s_Swerve,
                180.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        driverRIGHT.onTrue(new TurnToAngleCmd(
                s_Swerve,
                -90.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        driverLEFT.onTrue(new TurnToAngleCmd(
                s_Swerve,
                90.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        operatorUP.onTrue(new TurnToAngleCmd(
                s_Swerve,
                0.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        operatorDOWN.onTrue(new TurnToAngleCmd(
                s_Swerve,
                180.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        operatorRIGHT.onTrue(new TurnToAngleCmd(
                s_Swerve,
                -90.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
        operatorLEFT.onTrue(new TurnToAngleCmd(
                s_Swerve,
                90.0,
                () -> driver.getRawAxis(translationAxis),
                () -> driver.getRawAxis(strafeAxis),
                () -> speedReduction()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
