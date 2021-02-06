/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode". An OpMode is a 'program'
 * that runs in either the autonomous or the teleop period of an FTC match. The names of OpModes
 * appear on the menu of the FTC Driver Station. When an selection is made from the menu, the
 * corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * <p>This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot It
 * includes all the skeletal structure that all iterative OpModes contain.
 *
 * <p>Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new
 * name. Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode
 * list
 */
@TeleOp(name = "What Would I Do As A Robot", group = "Autonomous")
public class WhatIWouldDoAsARobot extends MasterAuto2021 {

  // Shooting Things
  private DcMotor leftShooter = null;
  private DcMotor rightShooter = null;

  // Wobble Goal Arm
  private DcMotor wobbleGoalMotor = null;

  public void InitializeShooterFlywheels() {

    // Get the Motors to Drive the Movement System
    leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
    rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

    // Set the direction of the Driving Motors
    // REASON: For the Mechanim Wheels to work simply, we Invert the Left Wheels.
    leftShooter.setDirection(DcMotor.Direction.FORWARD);
    rightShooter.setDirection(DcMotor.Direction.FORWARD);

    // Make it so that if there is no power to motors, they break.
    // REASON: Makes the robot stop much faster.
    leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Make the Motors so they run using the Encoder
    // REASON: This Leads To More Dependable Movement/ We are Now Able to Track Our Movement
    leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void InitWobbleArm() {

    // Get the Wobble Goal Motor from the Hardware Map
    wobbleGoalMotor = hardwareMap.get(DcMotor.class, "wobbleGoal");

    // Run the Motors Forward Instead of Reverse
    wobbleGoalMotor.setDirection(DcMotor.Direction.FORWARD);

    // Set the Motor Zero Power Mode to Break
    wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Run Without Encoder
    wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void Initialize() {

    // Tell the Baseline Code to Initialize (This will Initialize the Wheels and Gyro for Us)
    super.Initialize();

    // Initialize the Flywheels
    InitializeShooterFlywheels();

    // Initialize the Wobble Arm
    InitWobbleArm();
  }

  @Override
  public void runOpMode() {

    // Run the Initialize Function
    Initialize();

    // Move Forward
    driveFlat(1.0f, 0.0f);
    sleep(2000);

    // Turn Right
    turnFlat(90);
    sleep(1500);

    // Finish
    stop();
  }
}
