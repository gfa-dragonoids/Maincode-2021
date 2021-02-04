package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;



@Autonomous(name = "master", group = "Autonomous")
public class MasterAuto2021 extends LinearOpMode {

  public static final String VUFORIA_KEY =
      "AXDMU6L/////AAABmYsje6g+d0FouarmMJSceCUvoPLsXYHB38V7+MVCV//rzuYmaMR0aeKY+X1gyKROXD2HP/yqTdMoGKjNifE0TLgN3fUlxqF8CAejftyRLJXX7t1xBrivJKRDgDbQrX6I+6xe2ZcfInF2KnfQHOrlMh/i7M4RU6vzkIwKIzCwkV/SaMxAyYWpEngCIK+3ZelwN2uVIc0nXFNEXI2qVTaiAb7ffvbqzCBcxXrxCzbahSso5A/fD9f6FGsyMvVTQUzRaybT473gX+RJ1nPHyqjTscffYVyBGl0sAQ259VwLGwM+FE+ymehKO1shL9s1ITfaZaRdSWxzxvdS/e5xaavoXEw3ylD16GUnclpvw1s/ts7y";
  // endregion

  static final double TICKS_PER_ROTATION = 1120.0 * 0.75;
  static final double WHEEL_DIAMETER = 4; // bad units

  static final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_DIAMETER * Math.PI);
  static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
  static final double WHEEL_SPAN = 10.86614; // bad units

  WebcamName webcamName;

  BNO055IMU gyro;
  // variables.

  public VuforiaLocalizer vuforia;
  public TFObjectDetector tfod;

  public ElapsedTime runtime = new ElapsedTime();

  private DcMotor lf = null;
  private DcMotor rf = null;
  private DcMotor lb = null;
  private DcMotor rb = null;

  public void runOpMode() {}

  void initialize() {

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

    // Get the Gyro Position
    // NOTICE: We Don't Use the Gyro, but Its Here For the Hell Of It
    gyro = hardwareMap.get(BNO055IMU.class, "imu");

    // Wait For a 1/5 of a Second so The Robot can Have a Drink or Something
    // REASON: I'm Not Quite Sure, but It Was in the Old Code, so Why Not!
    sleep(200);
  }

  /** * */
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

    //
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

  void driveTime(double pwr, int time) {

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(pwr);

    sleep(time);

    halt();
  }

  void pivotTime(double pwr, int time) {

    rf.setPower(pwr);
    rb.setPower(pwr);
    lf.setPower(-pwr);
    lb.setPower(-pwr);

    sleep(time);

    halt();
  }

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

  void strafeTime(double pwr, int time) {

    rf.setPower(-pwr);
    rb.setPower(pwr);
    lf.setPower(pwr);
    lb.setPower(-pwr);

    sleep(time);

    halt();
  }

  /**
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

  /** @return the current gyro reading as an int from 0 to 360 degrees */
  int getGyro() {

    float firstAngle =
        gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            .firstAngle;
    int gyroAngle = (int) firstAngle;
    return gyroAngle;
  }
}
