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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="What Would I Do As A Robot", group="Autonomous")
public class WhatIWouldDoAsARobot extends MasterAuto2021
{
	
	// Global Variables
	// Global Variables
	static final double TICKS_PER_ROTATION = 1120.0 * 0.75;
	
	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	
	// drive
	private DcMotor lf = null;
	private DcMotor rf = null;
	private DcMotor lb = null;
	private DcMotor rb = null;
	
	// shooting thingys
	private DcMotor leftShooter = null;
	private DcMotor rightShooter = null;
	
	// Wait time
	long driveUntil = 0;
	
	public void initialize() {
		
		/// SETUP THE BOTTOM WHEELS
		// Get the Motors to Drive the Movement System
		lf = hardwareMap.get(DcMotor.class, "lf");
		lb = hardwareMap.get(DcMotor.class, "lb");
		rf = hardwareMap.get(DcMotor.class, "rf");
		rb = hardwareMap.get(DcMotor.class, "rb");
		
		// Set the direction of the Driving Motors
		// REASON: For the Mechanim Wheels to work simply, we Invert the Left Wheels.
		lf.setDirection(DcMotor.Direction.REVERSE);
		lb.setDirection(DcMotor.Direction.REVERSE);
		rf.setDirection(DcMotor.Direction.FORWARD);
		rb.setDirection(DcMotor.Direction.FORWARD);
		
		// Make it so that if there is no power to motors, they break.
		// REASON: Makes the robot stop much faster.
		rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// Make the Motors so they run using the Encoder
		// REASON: This Leads To More Dependable Movement/ We are Now Able to Track Our Movement
		lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		/// SETUP THE FLYWHEELS
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
		
		// LOG STATUS
		// Log the Status of the Robot and Tell the Driver that We Are Ready
		// REASON: It adds a bit more fun to the robot.
		// ALSO: Sorry Ethan, It was Too Much Fun.
		
		String[] possibleSayings = new String[]{"Let's roll.", "Ready To Rumble.", "Beep Boop.", "Taking Over The World", "About to Win The Contest"};
		telemetry.addData("Status", possibleSayings[(int)(Math.random() * possibleSayings.length)]);
		
	}
	
	@Override
	public void runOpMode() {
		
		initialize();
		
	}
	
}
