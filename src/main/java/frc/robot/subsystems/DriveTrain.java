/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDriveCommand;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
 
  // instantiate new motor controller objects
  public Spark rightBack = new Spark(RobotMap.rightBackP); 
  
  void setInverted(boolean rightBack) {
    
  }
  
  public Spark leftFront = new Spark(RobotMap.leftFrontP);
  public Spark leftBack = new Spark(RobotMap.leftBackP);
  public Spark rightFront = new Spark(RobotMap.rightFrontP);
  //connect all motors in mecanum drive
  public MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  
  //initiate manual drive
  public void manualDrive(double ySpeed, double xSpeed, double zRotation) {
    if (ySpeed < 0.2){
      ySpeed = 0;
    }

    if (xSpeed < 0.2){
      xSpeed = 0;
    }

    drive.driveCartesian(ySpeed, xSpeed, zRotation);
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualDriveCommand());
  }
}
