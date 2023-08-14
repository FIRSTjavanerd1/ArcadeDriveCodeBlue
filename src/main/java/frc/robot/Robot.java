// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//here we import the motors, joysticks, and other libraries.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;





//Here we declare our motors, joysticks and proggram arcade drive.

public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(1);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(4);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_drive = new Joystick(0);
  private final Joystick m_steer = new Joystick(0);
  private WPI_VictorSPX intake = new WPI_VictorSPX(5);
  private CANSparkMax flywheel = new CANSparkMax(6, MotorType.kBrushless);
  private WPI_VictorSPX intakeSpin = new WPI_VictorSPX(0);


  //here we declare our PID values
private double p = 0.0005;
private double i = 0.05;
private double d = 0.0;

//these are the positions, raised and lowered, of the intake.
private double raised = 0.0;
private double lowered = 800;


  private final PIDController intakeController = new PIDController(p, i, d);

  

private Encoder encoder = new Encoder(3, 2);



//all the code here happens when the robot is turned on
//here we invert the right motor

  @Override
  public void robotInit() {
   
    m_rightMotor.setInverted(true);


  
 

  }

  //this is teleop
  //code for arcade drive
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_drive.getY(), -m_drive.getX());


    //if button number four is pressed, the intake will be raised 
if(m_steer.getRawButtonPressed(4)){
  intake.set(intakeController.calculate(encoder.get(),raised));
}

//if button number two is pressed, the intake will be lowered  
else if(m_steer.getRawButtonPressed(2)){ 
  intake.set(intakeController.calculate(encoder.get(),lowered));
  }

//if button three is held the flywheel will run at full speed and half speed when button one is held.



  while(m_steer.getRawButtonPressed(3)){
    flywheel.set(1);
  }


  while(m_steer.getRawButtonPressed(1)){
    flywheel.set(0.5);
  }

  //if button five is pressed the intake will run forward, and backwards when button six is pressed.

  while(m_steer.getRawButtonPressed(5)){
    intakeSpin.set(1);
  }

  while(m_steer.getRawButtonPressed(6)){
    intakeSpin.set(-1);
  }

}

}

