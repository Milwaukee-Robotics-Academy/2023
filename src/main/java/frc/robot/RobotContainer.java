package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int intakeAxis = XboxController.Axis.kLeftTrigger.value;
    private final int outtakeAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final IntakeSubsystem intake = new IntakeSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        intake.setDefaultCommand(
            new Intakecommand(intake, () -> driver.getRawAxis(intakeAxis), () -> driver.getRawAxis(outtakeAxis))
        );

        // Configure the button bindings
        configureButtonBindings();
        updateAutoChoices();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Command Selected from Shuffleboard
        return autoChooser.getSelected();
    }


    public void updateAutoChoices() {
        if(DriverStation.getAlliance().equals(Alliance.Red)){
            try {
                autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
                autoChooser.addOption("Left start", new StartLeftPickupRed(s_Swerve));
                autoChooser.addOption("Center start", new StartCenterBalance(s_Swerve));
                Shuffleboard.getTab("Autonomous").add(autoChooser);
              } catch (NullPointerException ex) {
                autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
                DriverStation.reportError("auto choose NULL somewhere in RobotContainer.java", null);
              }
        }else if(DriverStation.getAlliance().equals(Alliance.Blue)){
            try {
                autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
                autoChooser.addOption("Left start", new StartLeftPickupBlue(s_Swerve));
                autoChooser.addOption("Center start", new StartCenterBalance(s_Swerve));
                Shuffleboard.getTab("Autonomous").add(autoChooser);
              } catch (NullPointerException ex) {
                autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
                DriverStation.reportError("auto choose NULL somewhere in RobotContainer.java", null);
              }
        }else {
            autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
        }

    }
}
