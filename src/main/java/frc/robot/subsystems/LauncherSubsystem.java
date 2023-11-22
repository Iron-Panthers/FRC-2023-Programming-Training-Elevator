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
  int time = 0;
  double motorPower;
  double launchMotorPower;
  //(motor).set(TalonFXControlMode.PercentOutput, #);
  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {

  }
  public setMotorSpeed(double secondMotorPower){
    launchMotorPower = secondMotorPower;
  }


  @Override
  //Periodic: Every 20 milliseconds it runs
  public void periodic() {
    motorPower = 0;
    if(time < 150){
    motorPower = -0.5;
    }
    if (time >= 150 && time < 300) {
      motorpower = launchMotorPower;
      motorPower = 0.75;
    } 
    if (time == 550){
      time = 0;
    }
  

    left_motor.set(TalonFXControlMode.PercentOutput, motorPower);
    right_motor.set(TalonFXControlMode.PercentOutput, motorPower);
    
    time++;
  }
} 