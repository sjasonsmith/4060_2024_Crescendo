// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. 
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem = new PoseEstimatorSubsystem(m_drivetrainSubsystem);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final SendableChooser<AutoMode> m_autonomousChooser = new SendableChooser<>();
  private final SendableChooser<Double> m_autonomousDelayChooser = new SendableChooser<>();

  private static final double MAX_JOYSTICK_TWIST_FIELD_RELATIVE = 0.5;



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      
  public void publishVariables() {
    SmartDashboard.putBoolean("Note Loaded", m_shooterSubsystem.isLoaded());
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure the autonomous command chooser
    configureAutonomousCommandChooser();
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
        () -> -modifyAxis(m_driverController.getRightX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * MAX_JOYSTICK_TWIST_FIELD_RELATIVE,
        m_poseEstimatorSubsystem::getCurrentRotation));

    // Map button Y to an instantaneous command that resets the Gyro heading on the Drivetrain Subsystem.
    m_driverController.y().onTrue(new InstantCommand(m_poseEstimatorSubsystem::resetFieldPosition, m_drivetrainSubsystem));

    // When left trigger is pulled, call startMotor. When it is released, stop the motor.
    // m_driverController.leftTrigger().whileTrue(Commands.startEnd(() -> m_shooterSubsystem.setShooterMotorSpeed(0.8), m_shooterSubsystem::stopShooterMotor, m_shooterSubsystem));
    // m_driverController.leftTrigger().whileTrue(new ShootCommand(m_shooterSubsystem));
    m_driverController.leftTrigger().whileTrue(
        GetSpinUpCommand()
        .andThen(Commands.startEnd(m_shooterSubsystem::feedShot, m_shooterSubsystem::stop, m_shooterSubsystem))
        );

    // When right shoulder is pressed, set Motor speed to -0.2. When it is released, stop the motor.
    // m_driverController.rightBumper().whileTrue(Commands.startEnd(() -> m_shooterSubsystem.setShooterMotorSpeed(-0.2),m_shooterSubsystem::stopShooterMotor, m_shooterSubsystem));
    m_driverController.rightBumper().whileTrue(Commands.startEnd(m_shooterSubsystem::feedIn, m_shooterSubsystem::stop, m_shooterSubsystem));

    // When D-Pad Up is pressed, extend the climber. When it is released, stop the motor.
    m_driverController.povUp().whileTrue(Commands.startEnd(m_climberSubsystem::retract, m_climberSubsystem::stopMotor, m_climberSubsystem));
    // When D-Pad Down is pressed, retract the climber. When it is released, stop the motor.
    m_driverController.povDown().whileTrue(Commands.startEnd(m_climberSubsystem::extend, m_climberSubsystem::stopMotor, m_climberSubsystem));

    // Map the X button to a sequential command that does the following:
    // 1. Roll the shooter formward at low power for 1 second
    // 2. Stop the shooter motors
    // 3. Drive the robot forward 0.5s at 20% speed 
    // 4. Driver the robot backward for 1s inches at 50% speed
    // 5. Stop the robot
    m_driverController.x().whileTrue(new SequentialCommandGroup(
        new RunCommand(() -> m_shooterSubsystem.setShooterMotorSpeed(0.2), m_shooterSubsystem).withTimeout(1),
        new InstantCommand(() -> m_shooterSubsystem.stopShooterMotor(), m_shooterSubsystem),
        GetAmpShootCommand(),
        new RunCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.1,0,0)), m_drivetrainSubsystem).withTimeout(3)
        // new RunCommand(() -> m_drivetrainSubsystem.stop(), m_drivetrainSubsystem).withTimeout(0.05),
        // new RunCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(-5.0,0,0)), m_drivetrainSubsystem).withTimeout(0.2)
    ).finallyDo(() -> {
        m_shooterSubsystem.stopShooterMotor();
        m_drivetrainSubsystem.stop();
    }));
}

    private InstantCommand SetFieldPoseCommand(double x, double y, double degrees) {
        return new InstantCommand(() -> m_poseEstimatorSubsystem.setCurrentPose(new Pose2d(x, y, Rotation2d.fromDegrees(degrees))));
    }

    private DriveToPoseCommand GoToMeters(double x, double y, double degrees) {
        return new DriveToPoseCommand(m_drivetrainSubsystem, m_poseEstimatorSubsystem, new Pose2d(x, y, Rotation2d.fromDegrees(degrees)));
    }

    private DriveToPoseCommand GoToMeters(double x, double y) {
        return GoToMeters(x, y, 0.0);
    }

    private final double AUTO_ANGLE_START_DEGREES_LEFT = 60.0;
    private final double AUTO_ANGLE_START_DEGREES_RIGHT = -AUTO_ANGLE_START_DEGREES_LEFT;
    private final double AUTO_STRAIGHT_FLEE_Y_METERS = 4.5;
    private final double AUTO_STRAIGHT_FLEE_X_METERS = 3.65; // 2.65; // 4.65;
    private final double AUTO_ANGLE_FLEE_Y_METERS = 3.75;
    private final double AUTO_ANGLE_FLEE_X_METERS = 4.0; //3.0; // 5.0;
    private final double AUTO_ANGLE_FLEEE_STRAIGHT_X_METERS = 3.0;
    private final double AUTO_AMP_START_DEGREES = 90.0;
    private final double AUTO_AMP_MOVE_1_X = 0.686;
    private final double AUTO_AMP_MOVE_2_Y = -0.5;
    private final double AUTO_AMP_MOVE_3_X = 0.5;

    private enum AutoMode {
        None,
        DriveForward_2m,
        DriveForward_5m,
        BlueSpeaker_Center_ShootAndFlee,
        BlueSpeaker_RightSide_ShootFlee,
        RedSpeaker_Center_ShootFlee,
        RedSpeaker_LeftSide_ShootFlee,
        Speaker_LeftSide_ShootStay,
        Speaker_Center_ShootStay,
        Speaker_RightSide_ShootStay,
        Speaker_LeftSide_ShootCross,
        Speaker_Center_ShootCross,
        Speaker_RightSide_ShootCross,
        ReturnToOrigin,
        ReturnToAngledOriginLeft,
        ReturnToAngledOriginRight,
        AmpPlaceAndStay,
        AmpPlaceAndFlee
    }

    private void configureAutonomousCommandChooser() {
        m_autonomousDelayChooser.setDefaultOption("0", 0.0);
        // Add options for 1-10 seconds
        for (int i = 1; i <= 10; i++) {
            m_autonomousDelayChooser.addOption(Integer.toString(i), ((double)i));
        }

        // Populate the autonomous command chooser
        m_autonomousChooser.setDefaultOption("None", AutoMode.None);
        m_autonomousChooser.addOption("ON LINE - Drive Forward 2m", AutoMode.DriveForward_2m);
        m_autonomousChooser.addOption("ON LINE - Drive Forward 5m", AutoMode.DriveForward_5m);
        m_autonomousChooser.addOption("BLUE SPEAKER (Center) - Shoot then flee to the RIGHT", AutoMode.BlueSpeaker_Center_ShootAndFlee);
        m_autonomousChooser.addOption("BLUE SPEAKER (Right Side) - Shoot then flee to the RIGHT", AutoMode.BlueSpeaker_RightSide_ShootFlee);
        m_autonomousChooser.addOption("RED SPEAKER (Center) - Shoot then flee to the LEFT", AutoMode.RedSpeaker_Center_ShootFlee);
        m_autonomousChooser.addOption("RED SPEAKER (Left Side) - Shoot then flee to the LEFT", AutoMode.RedSpeaker_LeftSide_ShootFlee);
        m_autonomousChooser.addOption("SPEAKER (Left Side) - Shoot Only", AutoMode.Speaker_LeftSide_ShootStay);
        m_autonomousChooser.addOption("SPEAKER (Center) Shoot Only", AutoMode.Speaker_Center_ShootStay);
        m_autonomousChooser.addOption("SPEAKER (Right Side) - Shoot Only", AutoMode.Speaker_RightSide_ShootStay);
        m_autonomousChooser.addOption("SPEAKER (Left Side) - Shoot and Cross", AutoMode.Speaker_LeftSide_ShootCross);
        m_autonomousChooser.addOption("SPEAKER (Center) Shoot and Cross", AutoMode.Speaker_Center_ShootCross);
        m_autonomousChooser.addOption("SPEAKER (Right Side) - Shoot and Cross", AutoMode.Speaker_RightSide_ShootCross);
        // m_autonomousChooser.addOption("AMP (Shooter Facing Amp, in corner) - Place and Stay", AutoMode.AmpPlaceAndStay);
        // m_autonomousChooser.addOption("AMP (Shooter Facing Amp, in corner) - Place and Flee", AutoMode.AmpPlaceAndFlee);
        // m_autonomousChooser.addOption("-----------", AutoMode.None);
        // m_autonomousChooser.addOption("DEBUG - Return to origin", AutoMode.ReturnToOrigin);
        // m_autonomousChooser.addOption("DEBUG - Return to origin (left side)", AutoMode.ReturnToAngledOriginLeft);
        // m_autonomousChooser.addOption("DEBUG - Return to origin (right side)", AutoMode.ReturnToAngledOriginRight);
         
        SmartDashboard.putData("Autonomous Selection", m_autonomousChooser);
        SmartDashboard.putData("Autonomous Delay", m_autonomousDelayChooser);
    }

    private Command GetSpinUpCommand() {
        return Commands.runEnd(m_shooterSubsystem::spinUp, m_shooterSubsystem::stop, m_shooterSubsystem).withTimeout(0.25);
    }

    private Command GetShootCommand() {
        return Commands.startEnd(m_shooterSubsystem::feedShot, m_shooterSubsystem::stop, m_shooterSubsystem).withTimeout(1.0);
    }

    private Command GetAmpShootCommand() {
        return Commands.startEnd(m_shooterSubsystem::feedAmp, m_shooterSubsystem::stop, m_shooterSubsystem).withTimeout(2);
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Combine requested delay and selected autonomous command
        double delay = m_autonomousDelayChooser.getSelected();
        AutoMode autoMode = m_autonomousChooser.getSelected();

        // Always initialise pose to the origin, so that commands only need to
        // specify if they need something different.
        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
            Commands.waitSeconds(delay));
        
        switch (autoMode) {
            case None:
                break;

            case DriveForward_2m:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, 0.0),
                    GoToMeters(2.0, 0)
                    );
                break;

            case DriveForward_5m:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, 0.0),
                    GoToMeters(5.0, 0)
                    );
                break;

            case BlueSpeaker_Center_ShootAndFlee:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, 0.0),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(0.6, -3.5),
                    GoToMeters(AUTO_STRAIGHT_FLEE_X_METERS, -AUTO_STRAIGHT_FLEE_Y_METERS)
                    );
                break;
    
            case BlueSpeaker_RightSide_ShootFlee:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_ANGLE_START_DEGREES_RIGHT),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(2.0, -2.67, AUTO_ANGLE_START_DEGREES_RIGHT),
                    GoToMeters(AUTO_ANGLE_FLEE_X_METERS, -AUTO_ANGLE_FLEE_Y_METERS, 0)
                    );
                break;
                   
            case RedSpeaker_Center_ShootFlee:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, 0.0),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(0.6, 3.5),
                    GoToMeters(AUTO_STRAIGHT_FLEE_X_METERS, AUTO_STRAIGHT_FLEE_Y_METERS)
                    );
                break;

            case RedSpeaker_LeftSide_ShootFlee:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_ANGLE_START_DEGREES_LEFT),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(2.0, 2.67, AUTO_ANGLE_START_DEGREES_LEFT),
                    GoToMeters(AUTO_ANGLE_FLEE_X_METERS, AUTO_ANGLE_FLEE_Y_METERS, 0)
                    );
                break;


            case Speaker_LeftSide_ShootStay:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_ANGLE_START_DEGREES_LEFT),
                    GetSpinUpCommand(),
                    GetShootCommand()
                    );
                break;

            case Speaker_Center_ShootStay:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, 0.0),
                    GetSpinUpCommand(),
                    GetShootCommand()
                    );
                break;

            case Speaker_RightSide_ShootStay:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_ANGLE_START_DEGREES_RIGHT),
                    GetSpinUpCommand(),
                    GetShootCommand()
                    );
                break;
 
   
            case Speaker_LeftSide_ShootCross:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_ANGLE_START_DEGREES_LEFT),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(AUTO_ANGLE_FLEEE_STRAIGHT_X_METERS, 0, AUTO_ANGLE_START_DEGREES_LEFT)
                    );
                break;
            
            case Speaker_Center_ShootCross:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, 0),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(AUTO_ANGLE_FLEEE_STRAIGHT_X_METERS, 0, 0)
                    );
                break;  
                
            case Speaker_RightSide_ShootCross:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_ANGLE_START_DEGREES_RIGHT),
                    GetSpinUpCommand(),
                    GetShootCommand(),
                    GoToMeters(AUTO_ANGLE_FLEEE_STRAIGHT_X_METERS, 0, AUTO_ANGLE_START_DEGREES_RIGHT)
                    );
                break;

                 
            case AmpPlaceAndStay:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_AMP_START_DEGREES),
                    GoToMeters(AUTO_AMP_MOVE_1_X, 0),
                    GoToMeters(AUTO_AMP_MOVE_1_X, AUTO_AMP_MOVE_2_Y).withTimeout(0.5),
                    GetAmpShootCommand()
                    );
                break;

            case AmpPlaceAndFlee:
                autoCommand.addCommands(
                    SetFieldPoseCommand(0, 0, AUTO_AMP_START_DEGREES),
                    GoToMeters(AUTO_AMP_MOVE_1_X, 0),
                    GoToMeters(AUTO_AMP_MOVE_1_X, AUTO_AMP_MOVE_2_Y).withTimeout(0.5),
                    GetAmpShootCommand(),
                    GoToMeters(AUTO_AMP_MOVE_3_X, 0, 0)
                    );
                break;

            // case ReturnToOrigin:
            //     autoCommand.addCommands(GoToMeters(0, 0));
            //     break;

            // case ReturnToAngledOriginLeft:
            //     autoCommand.addCommands(GoToMeters(0, 0, AUTO_ANGLE_START_DEGREES_LEFT));
            //     break;

            // case ReturnToAngledOriginRight:
            //     autoCommand.addCommands(GoToMeters(0, 0, AUTO_ANGLE_START_DEGREES_RIGHT));
            //     break;
            // default:
            //     break;
        }
        return autoCommand;
    }

  private static double deadband(double value, double deadband) {
    // Value has a range of -1 to 1
    if (Math.abs(value) < deadband) {
        return 0;
    } else {
      // Scale the value so that it is still -1 to 1, but scales with the deadband taken out
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
