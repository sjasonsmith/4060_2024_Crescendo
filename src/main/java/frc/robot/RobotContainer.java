// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. 
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();


  private static final double MAX_JOYSTICK_TWIST_FIELD_RELATIVE = 0.25;
    

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, true,
        () -> modifyAxis(-m_driverController.getLeftY())
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(m_driverController.getLeftX())
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyTwistAxis(m_driverController.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * MAX_JOYSTICK_TWIST_FIELD_RELATIVE));

    // Map button Y to an instantaneous command that resets the Gyro heading on the Drivetrain Subsystem.
    m_driverController.y().onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope(), m_drivetrainSubsystem));
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // When left trigger is pulled, call startMotor. When it is released, stop the motor.
    m_driverController.leftTrigger().whileTrue(Commands.startEnd(() -> m_shooterSubsystem.setMotorSpeed(0.8), m_shooterSubsystem::stopMotor, m_shooterSubsystem));

    // When right shoulder is pressed, set Motor speed to -0.2. When it is released, stop the motor.
    m_driverController.rightBumper().whileTrue(Commands.startEnd(() -> m_shooterSubsystem.setMotorSpeed(-0.2),m_shooterSubsystem::stopMotor, m_shooterSubsystem));

    // When D-Pad Up is pressed, extend the climber. When it is released, stop the motor.
    m_driverController.povUp().whileTrue(Commands.startEnd(m_climberSubsystem::extend, m_climberSubsystem::stopMotor, m_climberSubsystem));
    // When D-Pad Down is pressed, retract the climber. When it is released, stop the motor.
    m_driverController.povDown().whileTrue(Commands.startEnd(m_climberSubsystem::retract, m_climberSubsystem::stopMotor, m_climberSubsystem));
}

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  private static double modifyTwistAxis(double value) {
    // Deadband
    value = deadband(value, 0.25);

    // Square the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

}
