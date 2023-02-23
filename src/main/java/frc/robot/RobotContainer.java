// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.ExampleAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

// import com.pathplanner.lib.*;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
  private final Joystick driver;
  private final ButtonBox buttonBox;

  /* Drive Controls */
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 4;

  /* Driver Buttons */
  private final JoystickButton zeroGyro;
  private final JoystickButton autoBalance;

  /* Subsystems */
  private final SwerveBase swerveBase;

  // public Joystick getDriver() {
  // return driver;
  // }

  // public SwerveBase getSwerveSubsytem() {
  // return swerveBase;
  // }

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  private JoystickButton slow;

  public final POVButton UP;
  public final POVButton DOWN;
  public final POVButton LEFT;
  public final POVButton RIGHT;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driver = new Joystick(0);
    buttonBox = new ButtonBox(1);
    zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    autoBalance = new JoystickButton(driver, XboxController.Button.kX.value);
    slow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    UP = new POVButton(driver, 0);
    DOWN = new POVButton(driver, 180);
    LEFT = new POVButton(driver, 270);
    RIGHT = new POVButton(driver, 90);
    swerveBase = new SwerveBase();
    swerveBase.setDefaultCommand(
        new TeleopSwerve(
            swerveBase,
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            () -> speedReduction(),
            () -> !driver.getRawButton(XboxController.Button.kLeftBumper.value)));

    // Configure the button bindings
    configureButtonBindings();

    try {
      autoChooser.setDefaultOption("forward1m", swerveBase.followPathCmd("forward1m"));

      autoChooser.addOption("complex", swerveBase.followPathCmd("complex"));

      Shuffleboard.getTab("Autonomous").add(autoChooser);
    } catch (NullPointerException ex) {
      autoChooser.setDefaultOption("NULL nothing", new InstantCommand());
      DriverStation.reportError("auto choose NULL somewhere in RobotContainer.java", null);
    }
  }

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
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), anxd then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    zeroGyro.onTrue(new InstantCommand(() -> swerveBase.zeroHeading()));

    autoBalance.onTrue(new ProxyCommand(() -> new AutoBalanceCmd(swerveBase)));
    UP.onTrue(new TurnToAngleCmd(
        swerveBase,
        0.0,
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> speedReduction()));
    DOWN.onTrue(new TurnToAngleCmd(
        swerveBase,
        180.0,
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> speedReduction()));
    RIGHT.onTrue(new TurnToAngleCmd(
        swerveBase,
        -90.0,
        () -> driver.getRawAxis(translationAxis),
        () -> driver.getRawAxis(strafeAxis),
        () -> speedReduction()));
    LEFT.onTrue(new TurnToAngleCmd(
        swerveBase,
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
    try {
      // return autoChooser.getSelected();
      return new ExampleAuto(swerveBase);
    } catch (NullPointerException ex) {
      DriverStation.reportError("auto choose NULL somewhere in getAutonomousCommand in RobotContainer.java", null);
      return new InstantCommand();
    }
  }
}
