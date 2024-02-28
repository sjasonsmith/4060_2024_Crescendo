// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotorLeft =
            new CANSparkMax(CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
    private final CANSparkMax climberMotorRight =
            new CANSparkMax(CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);

    /** Creates a new ClimberSubsystem. */
    public ClimberSubsystem() {
        climberMotorLeft.restoreFactoryDefaults();
        climberMotorRight.restoreFactoryDefaults();
        climberMotorRight.follow(climberMotorLeft, true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void extend() {
        climberMotorLeft.set(0.2);
    }

    public void retract() {
        climberMotorLeft.set(-0.2);
    }

    public void stopMotor() {
        climberMotorLeft.stopMotor();
    }

}