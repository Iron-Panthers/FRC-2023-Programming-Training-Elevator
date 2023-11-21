// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {

  private TalonFX left_motor;
  left_motor = new TalonFX();
  private TalonFX right_motor;
  right_motor = new TalonFX();

  //(motor).set(TalonFXControlMode.PercentOutput, #);
  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {

  }

  @Override
  //Periodic: Every 20 milliseconds it runs
  public void periodic() {
    left_motor.set(TalonFXControlMode.PercentOutput, -0.5);
    right_motor.set(TalonFXControlMode.PercentOutput, -0.5);
    

  }
  
