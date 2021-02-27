package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "master", group = "Autonomous")
public class MasterAuto2021 extends LinearOpMode {

  static final double TICKS_PER_ROTATION = 1120.0 * 0.75;
  static final double WHEEL_DIAMETER = 4; // bad units

  static final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_DIAMETER * Math.PI);
  static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
  static final double WHEEL_SPAN = 10.86614; // bad units

  BNO055IMU gyro;

  public ElapsedTime runtime = new ElapsedTime();

  private DcMotor lf = null;
  private DcMotor rf = null;
  private DcMotor lb = null;
  private DcMotor rb = null;

  @Override
  public void runOpMode() {}

  /**
   *
   *
   * <h1>Initialize Robot</h1>
   *
   * <h3>Documentation Author: William M</h3>
   *
   * <p>This function initializes the robot's wheel motors. These are the motors that are on the
   * bottom of the robot. The software relationship to the wheels are 'rf', 'rb', 'lf', and 'lb'-
   * this corresponds to 'Right Front', 'Right Back', 'Left Front', and 'Left Back' respectively.
   *
   * @see MasterAuto2021 *
   */
  void InitializeWheels() {

    // Get the Motors
    // REASON: We Need to Tell the Robot Where its Motors Are so It Knows What We Are Talking About
    rf = hardwareMap.get(DcMotor.class, "rf");
    rb = hardwareMap.get(DcMotor.class, "rb");
    lf = hardwareMap.get(DcMotor.class, "lf");
    lb = hardwareMap.get(DcMotor.class, "lb");

    // Set the Motor Directions
    // REASON: Some of the Motors are Placed Backwards, so We Need to Account for that.
    rf.setDirection(DcMotor.Direction.FORWARD);
    rb.setDirection(DcMotor.Direction.FORWARD);
    lf.setDirection(DcMotor.Direction.REVERSE);
    lb.setDirection(DcMotor.Direction.REVERSE);

    // Set the Motor Behaviors
    // REASON: When We Stop Power on the Robot, the Robot Should Brake Completely
    rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /**
   *
   *
   * <h1>Initialize Gyroscope</h1>
   *
   * <h3>Documentation Author: William M</h3>
   *
   * <p>This function will initialize all of the gyroscope's data, but <b>will not</b> reset the
   * gyro at all.
   *
   * @see MasterAuto2021 *
   */
  void InitializeGyro() {

    gyro = hardwareMap.get(BNO055IMU.class, "imu");
  }

  /**
   * <h1>Initialize Gyroscope</h1>
   * <h3>Documentation Author: William M</h3>
   * <p>
   * This function will initialize all of the gyroscope's data, but <b>will not</b> reset the gyro
   * at all. The input to this function is a string that details the name of the gyroscope. If you
   * do not know the name of the Gyroscope, or if the gyroscope is named <code>'imu'</code>, then
   * just use the sister function, <code>InitializeGyro();<code/>
   * </p>
   * @param gyroName The name of the gyroscope that needs to be initialized.
   * @see MasterAuto2021
   * **/
  void InitializeGyro(String gyroName) {

    gyro = hardwareMap.get(BNO055IMU.class, "imu");
  }

  /**
   *
   *
   * <h1>Initialize Robot</h1>
   *
   * <h3>Documentation Author: William M</h3>
   *
   * <p>Initialize all of the Robot's code. This function has to be run at the start of the robot's
   * lifetime. If this function is not run, your robot will just start to throw errors instead of
   * moving around. This is due to the fact that the robot needs to know where the wheels and other
   * motors are located before it can start moving them.
   *
   * @see MasterAuto2021 *
   */
  void Initialize() {

    // Initialize the Wheels, Gyroscope, and Other Components
    InitializeWheels();
    InitializeGyro();

    // Wait For a 1/5 of a Second so The Robot can Have a Drink or Something
    // REASON: I'm Not Quite Sure, but It Was in the Old Code, so Why Not!
    sleep(200);

    // Log the Robot's Current Status
    String[] possibleSayings =
        new String[] {
          "Let's roll.",
          "Ready To Rumble.",
          "Beep Boop.",
          "Taking Over The World",
          "About to Win The Contest"
        };
    telemetry.addData("Status", possibleSayings[(int) (Math.random() * possibleSayings.length)]);
  }

  /**
   *
   *
   * <h1>Reset Robot</h1>
   *
   * <h3>Documentation Author: William M</h3>
   *
   * <p>This function will reset all of the robot's motors. This will also reset the encoder values,
   * as well as the actual motors. I am not sure if this will move the motors, but I am like 50%
   * sure that nothing will happen, so it should be good. So, if your year is using encoders with
   * variables instead of just encoders or running based on time, first of all, thank you for being
   * smart with how you programmed the robot, and secondly you will need to reset the variable
   * values below as well.
   *
   * @see MasterAuto2021 *
   */
  void reset() {

    // Stop and Reset All of the Encoders
    rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Set a Target Position for the Motors of Zero
    rf.setTargetPosition(0);
    rb.setTargetPosition(0);
    lf.setTargetPosition(0);
    lb.setTargetPosition(0);

    // Runs the Current Motors to the Position Specified by .setTargetPosition(0)
    rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  void halt() {

    rf.setPower(0);
    rb.setPower(0);
    lf.setPower(0);
    lb.setPower(0);
  }

  /**
   *
   *
   * <h1>Flat Drive Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Drives the robot as a fxn of time
   *
   * <p>Note: Due to the wheel configuration, this function deals with strafing, not rotating.
   *
   * @param horizontal The horizontal power, range is -1 to 1
   * @param vertical The power that is needed to move on the vertical axis. Range is -1 to 1.
   */
  void driveFlat(float vertical, float horizontal) {

    lf.setPower(Range.clip(vertical + horizontal, -1.0, 1.0));
    lb.setPower(Range.clip(vertical - horizontal, -1.0, 1.0));
    rf.setPower(Range.clip(vertical - horizontal, -1.0, 1.0));
    rb.setPower(Range.clip(vertical + horizontal, -1.0, 1.0));

    return;
  }

  /**
   *
   *
   * <h1>Flat Turn Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Rotates the robot
   *
   * <p>Note: This function only roates the robot to the specified angle
   *
   * @param angle The angle that the robot will rotate to, in degrees.
   */
  void turnFlat(float angle) {

    lf.setPower(Range.clip(angle / 180, -1.0, 1.0));
    lb.setPower(Range.clip(angle / 180, -1.0, 1.0));
    rf.setPower(Range.clip(angle / 180, -1.0, 1.0));
    rb.setPower(Range.clip(angle / 180, -1.0, 1.0));

    return;
  }

  /**
   *
   *
   * <h1>driveTime Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Drives the robot for a given power and given time.
   *
   * <p>Note that the power is not translated, and applies the same amout of power to all four
   * motors equally. Therefore, this function is only really useful for robots with a four wheel
   * design and basic wheels.
   *
   * <p>it may be better to use the driveFlat function, which starts on line 196.
   *
   * @param pwr The power that will be applied to each of the four robots motors.
   * @param time The time that the robot will apply the power, in milliseconds.
   */
  void driveTime(double pwr, int time) {

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(pwr);

    sleep(time);

    halt();
  }

  /**
   *
   *
   * <h1>pivotTime function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Pivots the robot for a specific power and time, rotating to the right by default (with a
   * positive power input)
   *
   * <p>Note that, like with the driveTime function, this design is best for four wheel designs with
   * simple wheels, the turnFlat function may be better (starts on line 213)
   *
   * @param pwr The power that the robot will rotate with.
   * @param time The time that the robot will rotate for.
   */
  void pivotTime(double pwr, int time) {

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(-pwr);
    lb.setPower(-pwr);

    sleep(time);

    halt();
  }

  /**
   *
   *
   * <h1>drive Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Drives the robot a given number of tiles, the ticks that this takes is defined in the
   * beginning of this file. We are using some tiles that we have, but you may not. You could update
   * this function to use inches instead, or go to the beginning of this file to update the ticks
   * per tile, but please update the documentation accordingly. Either search for or replace
   * 'TICKS_PER_TILE' with 'TICKS_PER_INCH' in order to change this function.
   *
   * @param distance The distance that the robot will travel, measured in tiles.
   * @param pwr The amount of power that the robot will use.
   */
  void drive(double distance, double pwr) {

    reset();
    int target = (int) (distance * TICKS_PER_TILE);

    pwr = Math.abs(pwr);

    rf.setTargetPosition(target);
    rb.setTargetPosition(target);
    lf.setTargetPosition(target);
    lb.setTargetPosition(target);

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(pwr);

    while (opModeIsActive() && rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) {

      telemetry.addData("rfpos", rf.getCurrentPosition());
      telemetry.addData("rbpos", rb.getCurrentPosition());
      telemetry.addData("lfpos", lf.getCurrentPosition());
      telemetry.addData("lbpos", lb.getCurrentPosition());

      telemetry.addData("rfpow", rf.getPower());
      telemetry.addData("rbpow", rb.getPower());
      telemetry.addData("lfpow", lf.getPower());
      telemetry.addData("lbpow", lb.getPower());

      telemetry.addData("complete", 1);
      telemetry.update();
    }

    halt();
    reset();
  }

  /**
   *
   *
   * <h1>drive Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>The same as the other drive function, but with linear acceleration. Just delete the third
   * variable in order to see the documentation.
   *
   * @param distance to move forward in tiles
   * @param pwr to give the motors, from 0.0 to 1.0
   * @param ramp the distance in inches to accelerate linearly through
   */
  void drive(double distance, double pwr, int ramp) {

    reset();
    int target = (int) (distance * TICKS_PER_TILE);

    double fpwr = Math.abs(pwr);

    rf.setTargetPosition(target);
    rb.setTargetPosition(target);
    lf.setTargetPosition(target);
    lb.setTargetPosition(target);

    while (opModeIsActive() && rf.isBusy()) {

      if (rf.getCurrentPosition() >= ramp * TICKS_PER_TILE / 24) {
        pwr = fpwr;
      } else {
        pwr = fpwr * (rf.getCurrentPosition() + 50) / (ramp * TICKS_PER_TILE / 24);
      }

      rf.setPower(pwr);
      rb.setPower(pwr);
      lf.setPower(pwr);
      lb.setPower(pwr);

      telemetry.addData("rfpos", rf.getCurrentPosition());
      telemetry.addData("rbpos", rb.getCurrentPosition());
      telemetry.addData("lfpos", lf.getCurrentPosition());
      telemetry.addData("lbpos", lb.getCurrentPosition());

      telemetry.addData("rfpow", rf.getPower());
      telemetry.addData("rbpow", rb.getPower());
      telemetry.addData("lfpow", lf.getPower());
      telemetry.addData("lbpow", lb.getPower());
      telemetry.update();
    }

    reset();
    halt();
  }

  /**
   *
   *
   * <h1>swivelpivot Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Pivots the robot by a certain angle.
   *
   * <p>To be honest I'm not sure if this function or turnFlat would be better, so try both and see
   * which one works for you. At least there's no weird units this time, right?
   *
   * <p>To be honest I didn't write this function, William M did, so I'm not exacly sure how the
   * robot turns, so try it out and see if it works. I will try to get him to write a better
   * explanation here.
   *
   * <p>If you look at the function below this one, it is much simpler, so that, along with how the
   * power is distributed, leads me to believe that this one will move the robot in a circle, versus
   * rotating it in a point.
   *
   * @param angle The angle that the robot will turn.
   * @param pwr The power that the robot will use.
   */
  void swivelpivot(double angle, double pwr) {

    reset();
    double distance =
        0.8
            * Math.PI
            * (WHEEL_SPAN)
            * angle
            / 255; // remember learning s = r*theta? it's back to haunt you.
    int target = (int) (distance * TICKS_PER_INCH);
    pwr = Math.abs(pwr);

    rf.setTargetPosition(target);
    rb.setTargetPosition((int) (target * .5547));
    lf.setTargetPosition(-target);
    lb.setTargetPosition(-(int) (target * .5547));
    rf.setPower(pwr);
    rb.setPower(pwr * .5547);
    lf.setPower(pwr);
    lb.setPower(pwr * .5547);

    while (opModeIsActive() && rf.isBusy()) {

      telemetry.addData("rfpos", rf.getCurrentPosition());
      telemetry.addData("rbpos", rb.getCurrentPosition());
      telemetry.addData("lfpos", lf.getCurrentPosition());
      telemetry.addData("lbpos", lb.getCurrentPosition());

      telemetry.addData("rfpow", rf.getPower());
      telemetry.addData("rbpow", rb.getPower());
      telemetry.addData("lfpow", lf.getPower());
      telemetry.addData("lbpow", lb.getPower());
      telemetry.update();
    }
    halt();
  }

  /**
   *
   *
   * <h1>pivot Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>This function pivots the robot on a point to a specific angle, not much else to say here.
   *
   * <p>This function defaults to the right, so an angle input of '90' will rotate the robot 90
   * degrees to the right of the current orientation/
   *
   * @param angle The angle that the robot will rotate.
   * @param pwr The power that the robot will use.
   */
  void pivot(double angle, double pwr) {
    reset();
    double distance =
        Math.PI
            * (WHEEL_SPAN / 2)
            * angle
            / 180; // remember learning s = r*theta? it's back to haunt you.
    int target = (int) (distance * TICKS_PER_INCH);
    pwr = Math.abs(pwr);

    rf.setTargetPosition(target);
    rb.setTargetPosition(target);
    lf.setTargetPosition(-target);
    lb.setTargetPosition(-target);

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(pwr);

    while (opModeIsActive() && rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) {
      telemetry.addData("rfD", rf.getCurrentPosition());
      telemetry.addData("rbD", rb.getCurrentPosition());
      telemetry.addData("lfD", lf.getCurrentPosition());
      telemetry.addData("lbD", lb.getCurrentPosition());

      telemetry.addData("rfP", rf.getPower());
      telemetry.addData("rbP", rb.getPower());
      telemetry.addData("lfP", lf.getPower());
      telemetry.addData("lbP", lb.getPower());
      telemetry.update();
    }
    halt();
    reset();
  }
  /**
   *
   *
   * <h1>strafeAC Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Note that, per Will's comments on this function, you may end up over 10 degrees off course
   * if you use this function. However, I am unable to test that at the moment, so this may be
   * incorrect.
   *
   * <p>This function will strafe the robot to the left or the right. You have to test to see if a
   * positive input will strafe the robot to the left or the right, or I may update this, if I can
   * test the robot. Or, you could update the documentation, if you feel so inclined.
   *
   * <p>The 'urgency' float (inside of the function) is what determines the, well, urgency. This is
   * also probably what send the robot off course, so you should probably use the strafe function
   * (the one below this one) if you want accuracy, but no direct control over urgency.
   *
   * <p>Note that the power of the robot also controls speed, so the urgency float is just another
   * way to directly control speed, without having to worry about increased power.
   *
   * @param distance The distance that the robot will strafe.
   * @param pwr The power that the robot will use.
   */
  void strafeAC(double distance, double pwr) {
    reset();
    int target = (int) (distance * TICKS_PER_TILE);

    float urgency = 0.15f; // remember that we may end up more than 10 degrees off course.

    float initial = getGyro();
    float current;

    rf.setTargetPosition(-target);
    rb.setTargetPosition(target);
    lf.setTargetPosition(target);
    lb.setTargetPosition(-target);

    while (opModeIsActive() && rf.isBusy()) {

      current = getGyro();
      float d = initial - current;

      rf.setPower(pwr - d * urgency * pwr);
      rb.setPower(pwr + d * urgency * pwr);
      lf.setPower(pwr - d * urgency * pwr);
      lb.setPower(pwr + d * urgency * pwr);

      telemetry.addData("R f pwr", rf.getPower());
      telemetry.addData("R b pwr", rb.getPower());
      telemetry.addData("L f pwr", lf.getPower());
      telemetry.addData("L b pwr", lb.getPower());

      telemetry.addData("d", d);
      telemetry.addData("p", d * urgency);

      telemetry.update();
    }

    halt();
    reset();
  }

  /**
   *
   *
   * <h1>Strafe Function</h1>
   *
   * <p>A basic strafe function, that strafes the robot. I am unsure if a positive direction input
   * will result in left or right motion, so please experiment and upade the documentation, or I may
   * do so myself at a later date.
   *
   * @param distance The distance that the robot will strafe.
   * @param pwr The power that the robot will use.
   */
  void strafe(double distance, double pwr) {

    reset();
    int target = (int) (distance * TICKS_PER_TILE);

    pwr = Math.abs(pwr);

    rf.setTargetPosition(-target);
    rb.setTargetPosition(target);
    lf.setTargetPosition(target);
    lb.setTargetPosition(-target);

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(pwr);

    while (opModeIsActive() && rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy()) {

      telemetry.addData("rfpos", rf.getCurrentPosition());
      telemetry.addData("rbpos", rb.getCurrentPosition());
      telemetry.addData("lfpos", lf.getCurrentPosition());
      telemetry.addData("lbpos", lb.getCurrentPosition());

      telemetry.addData("rfpow", rf.getPower());
      telemetry.addData("rbpow", rb.getPower());
      telemetry.addData("lfpow", lf.getPower());
      telemetry.addData("lbpow", lb.getPower());

      telemetry.addData("complete", 1);
      telemetry.update();
    }

    halt();
    reset();
  }

  /**
   *
   *
   * <h1>strafeTime Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Will strafe the robot for a given power and time.
   *
   * <p>Note that there is no translation of distance, so you will have to experiment to find what
   * power and time will move the robot the proper distance.
   *
   * @param pwr
   * @param time
   */
  void strafeTime(double pwr, int time) {

    rf.setPower(-pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(-pwr);

    sleep(time);

    halt();
  }

  /**
   *
   *
   * <h1>driveAC Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Will drive the robot a certain distance, using a certain power. Change the urgency variable
   * to change the speed directly.
   *
   * <p>Note that William M wrote this code, so I am unsure exactly what this code does (same with
   * all of the AC functions, so be careful and experiment properly.
   *
   * @param distance distance. may be positive or negative
   * @param pwr POSITIVE power
   */
  void driveAC(double distance, double pwr) {

    reset();
    int target = (int) (distance * TICKS_PER_TILE);

    float urgency = 0.05f; // remember that we may end up more than 10 degrees off course.

    pwr = Math.abs(pwr);

    float initial = getGyro();
    float current;

    rf.setTargetPosition(target);
    rb.setTargetPosition(target);
    lf.setTargetPosition(target);
    lb.setTargetPosition(target);

    while (opModeIsActive() && rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy()) {

      current = getGyro();
      float d = initial - current;

      rf.setPower(pwr + d * urgency * pwr);
      rb.setPower(pwr + d * urgency * pwr);
      lf.setPower(pwr - d * urgency * pwr);
      lb.setPower(pwr - d * urgency * pwr);

      telemetry.addData("R pwr", rf.getPower());
      telemetry.addData("L pwr", lf.getPower());

      telemetry.addData("d", d);
      telemetry.addData("p", d * urgency);

      telemetry.addData("tgt p", target);
      telemetry.addData(
          "max p",
          Math.max(
              Math.max(rf.getCurrentPosition(), lf.getCurrentPosition()),
              Math.max(rb.getCurrentPosition(), lb.getCurrentPosition())));

      telemetry.update();
    }

    halt();
    reset();
  }

  /**
   *
   *
   * <h1>turnAC Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Will turn the robot a certain degree to the right from ghe current orientation, using a
   * certain power.
   *
   * <p>Note that this function is 'weird', I will have to work more with a completed robot in order
   * to fully determine what this function does, or just ask Will.
   *
   * @param angle
   * @param pwr
   */
  void turnAC(int angle, double pwr) {
    reset();

    if (angle == 0) {
      return;
    }

    pwr = Math.abs(pwr);

    int tolerance = 15;

    int target = getGyro() + angle;

    if (target > 180) target = target - 360;
    if (target < -180) target = target + 360;

    int current = getGyro();

    sleep(500);

    rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    int sign = angle / Math.abs(angle);

    rf.setPower(sign * pwr);
    rb.setPower(sign * pwr);
    lf.setPower(-sign * pwr);
    lb.setPower(-sign * pwr);

    while (opModeIsActive() && Math.abs(target - current) > tolerance * pwr + 5) {

      current = getGyro();

      telemetry.addData("target", target);
      telemetry.addData("current", current);
      telemetry.addData("sign", sign);
      telemetry.addData("delta", Math.abs(target - current));
      telemetry.addData("tolerance", tolerance);

      telemetry.update();
    }

    halt();

    sleep(300);

    current = getGyro();

    rf.setPower(-sign * pwr * 0.2);
    rb.setPower(-sign * pwr * 0.2);
    lf.setPower(sign * pwr * 0.2);
    lb.setPower(sign * pwr * 0.2);

    while (opModeIsActive() && Math.abs(target - current) > 5) {

      current = getGyro();

      telemetry.addData("target", target);
      telemetry.addData("current", current);
      telemetry.addData("sign", sign);
      telemetry.addData("delta", Math.abs(target - current));
      telemetry.addData("tolerance", tolerance);

      telemetry.update();
    }

    halt();
  }

  /**
   *
   *
   * <h1>turnAC_debug Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Use this to debug the turnAC function. I will have to experiment with a robot to determine
   * exactly how this function differs from the regular turnAC function.
   *
   * @param angle The angle that the robot will turn.
   * @param pwr The power that the robot will use.
   */
  void turnAC_debug(int angle, double pwr) {
    reset();

    if (angle == 0) {
      return;
    }

    pwr = Math.abs(pwr);

    int tolerance = 15;

    int target = getGyro() + angle;

    if (target > 180) target = target - 360;
    if (target < -180) target = target + 360;

    int current = getGyro();

    sleep(500);

    rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    int sign = angle / Math.abs(angle);

    rf.setPower(sign * pwr);
    rb.setPower(sign * pwr);
    lf.setPower(-sign * pwr);
    lb.setPower(-sign * pwr);

    while (opModeIsActive() && Math.abs(target - current) > tolerance * pwr + 5) {
      current = getGyro();

      // region telemetry

      telemetry.addData("target", target);
      telemetry.addData("current", current);
      telemetry.addData("sign", sign);
      telemetry.addData("delta", Math.abs(target - current));
      telemetry.addData("tolerance", tolerance);

      telemetry.update();
      // endregion
    }

    halt();

    while (opModeIsActive()) {

      current = getGyro();

      telemetry.addData("target", target);
      telemetry.addData("current", current);
      telemetry.addData("sign", sign);
      telemetry.addData("delta", Math.abs(target - current));
      telemetry.addData("tolerance", tolerance);

      telemetry.update();
    }
  }

  /**
   *
   *
   * <h1>turnABC Function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Turns the robot using the gyro.
   *
   * <p>Like usual, I do not have access to the robot so cannot provide more information at this
   * time. Once the robot is fixed I will ask if I can use it to update the documentation.
   *
   * @param angle The angle that the robot will turn.
   * @param pwr The power that the robot will use.
   */
  void turnACB(int angle, double pwr) {
    reset();

    if (angle == 0) {
      return;
    }

    pwr = Math.abs(pwr);

    int tolerance = 15;

    int target = getGyro() + angle;

    if (target > 180) target = target - 360;
    if (target < -180) target = target + 360;

    int current = getGyro();

    sleep(500);

    rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    int sign = angle / Math.abs(angle);

    rf.setPower(sign * pwr);
    rb.setPower(sign * pwr);
    lf.setPower(-sign * pwr);
    lb.setPower(-sign * pwr);

    while (opModeIsActive() && Math.abs(target - current) > tolerance * pwr + 5) {

      current = getGyro();

      telemetry.addData("target", target);
      telemetry.addData("current", current);
      telemetry.addData("sign", sign);
      telemetry.addData("delta", Math.abs(target - current));
      telemetry.addData("tolerance", tolerance);

      telemetry.update();
    }

    halt();
  }

  /**
   *
   *
   * <h1>getGyro function</h1>
   *
   * <h3>Documentation Author: Alden G</h3>
   *
   * <p>Returns the current angle from a set axis point, which is probably the orientation that the
   * robot was originally in, but that is unconfirmed.
   *
   * @return the current gyro reading as an int from 0 to 360 degrees
   */
  int getGyro() {

    float firstAngle =
        gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            .firstAngle;
    int gyroAngle = (int) firstAngle;
    return gyroAngle;
  }
}
