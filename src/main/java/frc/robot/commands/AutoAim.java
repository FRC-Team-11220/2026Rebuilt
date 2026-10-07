package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class AutoAim extends Command{

    CANDriveSubsystem driveSubsystem;
    double averageX = 0;
    int fiducialCount = 0;

    public AutoAim(CANDriveSubsystem driveSystem){
        addRequirements(driveSystem);
        driveSubsystem = driveSystem;
    }

  //called whem this command is first schedueled 
  @Override
  public void initialize() {
  }

  //called every time the scheduler runs while this command is schedueled 
  @Override
  public void execute() {
    averageX = 0;
    fiducialCount = 0;

    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (RawFiducial fiducial : fiducials){
        averageX += fiducial.txnc;
        fiducialCount++;
    }

    averageX /= fiducialCount;

    System.out.println(averageX);

    if(Math.abs(averageX) >= 3){
        if(averageX > 0){
        driveSubsystem.driveArcade(0, 1);
        }else{
        driveSubsystem.driveArcade(0, -1);
        }
    }
    /*pseudo code
    average april tag x position and store it in tagAverage
    if tagAverage is positive (right half of camera)
        rotate left
    if tagAverage is negative (left half of camera)
        rotate right
    if Math.abs(tagAverage < 0.1) (near center)
        do nothing
    */


  }

  //if the command is interupted, set the motors to stop moving to prevent catastrophic failure
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  //I'm not sure why this in necessary but it is likely important
  @Override
  public boolean isFinished() {
    return false;
  }
}
