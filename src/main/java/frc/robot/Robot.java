/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.vision.Vision_Tracking;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    // Vision Controller
    private final Vision_Tracking vision = new Vision_Tracking(0,60/1280);

    // Robot Differential Drive
    private final Drivetrain drivetrain = new Drivetrain(this, vision);

    // Human Controls
    private final Joystick drive_control = new Joystick(0);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
      vision.start();
    }

    /**
     * This function is run once each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once each time the robot enters teleoperated mode.
     */
    @Override
    public void teleopInit() {
    }

    /**
     * This function is called periodically during teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
        drivetrain.set(drive_control.getRawAxis(1), drive_control.getRawAxis(5));
        
        if(drive_control.getRawButton(0)){
            drivetrain.enable_target_tracking(0);
        }else{
            drivetrain.disable_target_tracking();
        }

        drivetrain.update();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
