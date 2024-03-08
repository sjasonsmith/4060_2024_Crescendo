// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotorLeft =
            new CANSparkMax(SHOOTER_WHEEL_LEFT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax shooterMotorRight =
            new CANSparkMax(SHOOTER_WHEEL_RIGHT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax feederMotor =
            new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
    
    
    // Add the note loaded switch
    private final DigitalInput shooterLoadedSwitch = new DigitalInput(NOTE_LOADED_SWITCH);

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        shooterMotorLeft.restoreFactoryDefaults();
        shooterMotorRight.restoreFactoryDefaults();
        shooterMotorRight.follow(shooterMotorLeft, true);
        feederMotor.restoreFactoryDefaults();
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            /* one-time action goes here */
        });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setShooterMotorSpeed(double power) {
        shooterMotorLeft.set(power);
    }

    public void shoot() {
        shooterMotorLeft.set(0.5);
    }

    public void stopShooterMotor() {
        shooterMotorLeft.stopMotor();
    }

    public boolean isLoaded() {
        return !shooterLoadedSwitch.get();
    }

    private static double SHOOTER_INTAKE_SPEED = -0.2;
    private static double SHOOTER_SHOOT_SPEED = 1.0;
    private static double FEEDER_INTAKE_SPEED = -0.25;
    private static double FEEDER_SHOOT_SPEED = 1.0;
    private static double FEEDER_AMP_SPEED = 0.25;
    private static double SHOOTER_AMP_SPEED = 0.1;

    public void feedIn() {
        shooterMotorLeft.set(SHOOTER_INTAKE_SPEED); 
        feederMotor.set(FEEDER_INTAKE_SPEED);
    }

    public void spinUp() {
        shooterMotorLeft.set(SHOOTER_SHOOT_SPEED);
    }

    public void feedShot() {
        shooterMotorLeft.set(SHOOTER_SHOOT_SPEED);
        feederMotor.set(FEEDER_SHOOT_SPEED);
    }

    public void feedAmp() {
        shooterMotorLeft.set(SHOOTER_AMP_SPEED);
        feederMotor.set(FEEDER_AMP_SPEED);
    }

    public void stop() {
        shooterMotorLeft.stopMotor();
        feederMotor.stopMotor();
    }

}
