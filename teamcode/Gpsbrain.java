package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.reflect.Array;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Chomp;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Find;
import org.firstinspires.ftc.teamcode.SciLift;

//The meat. Our magnum opus. The incredible autonomous program.
//We control the robot using special commands stored in a list.
//More information on how we control the bot here:
//https://github.com/HHS-FTC-Robotics-Team/Team-Resources/wiki/States

public class Gpsbrain extends LinearOpMode {

  public String state = "rest";
  public Boolean turning = false;
  boolean angleIsSeeked = false;
  double seekAngle = 0;
  double seekDist = 0;
  Drive d = null;
  double globalx = 0;
  double globaly = 0;
  double startx = 0;
  double starty = 0;
  double globala = 0;
  double dx = 0;
  double dy = 0;
  double theta = 0;
  double dtheta = 0;
  double travelled = 0;
  public double goalclicks = 0;
  public double relativex = 0;
  public double relativey = 0;
  double startclicks = 0;
  double liftgoalclicks = 0;
  double liftstartclicks = 0;
  public int count = 0;

  //List of different command sequences===================================================================================

  //public String[] states = new String[]{"lift", "rest"};
  //private double[] args = new double[]{-1000, 0};
  //private boolean[] isArgs = new boolean[]{true, false};

  // Collect
  // public String[] states = new String[]{"forward", "seek","turn","collect","forward","strafeRight","out","rest"};
  // private double[] args = new double[]{-1000, 0, 180, 0, -2500,7000, 0,0};
  // private boolean[] isArgs = new boolean[]{true, false, true, false, true, true, false,false};

  // Park
  // public String[] states = new String[]{"forward", "strafeRight"};
  // private double[] args = new double[]{-1000, 5600};
  // private boolean[] isArgs = new boolean[]{true, true};

  // Just collecting
  public String[] states = new String[]{"init", "collect", "rest"};
  private long[] args = new long[]{0, 0, 0};
  private boolean[] isArgs = new boolean[]{false, false, false};

  // Testing global x and y
  // public String[] states = new String[]{"init", "collect", "strafeTo", "rest"};
  // private double[] args = new double[]{0, 0, 5000, 0};
  // private boolean[] isArgs = new boolean[]{false, true, true, false};

  // public String[] states = new String[]   {"init","forward","collect","forward","strafeTo", "out", "strafeTo","rest"};
  // private double[] args = new double[]    {0, 500, 0, 1400, -6000, 0, -1000, 0};
  // private boolean[] isArgs = new boolean[]{false, true, false, true, true,false, true, false};

  // public String[] states = new String[]   {"init","strafeTo","strafeTo","turn","strafeTo","strafeTo","turn","rest"};
  // private double[] args = new double[]    {0,800,0,180,800,0,180,0};
  // private boolean[] isArgs = new boolean[]{false, true,true,true, true,true,true,false};

  //Wait and Park
  // public String[] states = new String[]{"sleep", "forward", "strafeLeft"};
  // private long[] args = new long[]{24000,-500, 5600};
  // private boolean[] isArgs = new boolean[]{true, true, true};

  //======================================================================================================================

  private BNO055IMU imu = null;
  private Orientation lastAngles = new Orientation();
  private double globalAngle, power = 0.30, correction;
  public SciLift lift = null;
  Chomp collect = null;
  Find f = null;

  public Gpsbrain(Drive drive, BNO055IMU acc, Chomp c, Find find, SciLift scl) {
    d = drive;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      parameters.mode                = BNO055IMU.SensorMode.IMU;
      parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
      parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
      parameters.loggingEnabled      = false;
    imu = acc;
    imu.initialize(parameters);
    collect = c;
    f = find;
    lift = scl;
  }

  public void pop() {
    count = count + 1;
  }

  public void update() {
    if(states[count] == "init") {
      globalx = 0;
      globaly = 0;
      globala = getAngle(); //we will always correct to this angle
      d.resetEncoderlf();
      if (collect.setPos("up")) { pop(); }
      // pop();
    }
    if(states[count] == "rest") {
      d.setPower(0, 0, 0, 0);
      lift.motor.setPower(0);
      collect.setPos("down");
    }
    if(states[count] == "turn") {
      if (isArgs[count]) {
        this.turn(args[count]);
        isArgs[count] = false;
      }
      this.turn();
    }
    if(states[count] == "forward"){
      if (isArgs[count]) {
        this.forward(args[count]);
        isArgs[count] = false;
      }
      this.forward();
    }
    if(states[count] == "forwardTo"){
      if (isArgs[count]) {
        this.forwardTo(args[count]);
        isArgs[count] = false;
      }
      this.forward();
    }
    if(states[count] == "strafeTo"){
      if (isArgs[count]) {
        this.strafeTo(args[count]);
        isArgs[count] = false;
      }
      this.strafe();
    }
    if(states[count] == "seek") {
      //angleIsSeeked = false;
      if(!angleIsSeeked) {
        double[] result = f.findSkystoneAngle();
        seekAngle = result[0];
        if(result[1] > 0) {
          seekDist = 6000/Math.tan(seekAngle);
          //strafe(seekDist);
          angleIsSeeked = true;
          // if(seekAngle < 0) {
          //   seekDist = seekDist * -1;
          // }
          strafe(seekDist);
        }
      } else {
        //seekAngle =-500;
        strafe();
      }
      // double angle = seekAngle;
      // double dist = seekDist;
      // if(angle < 10 && angle > -10) {
      //   globalx += d.getClickslf();
      //   d.resetEncoderlf();
      //   angleIsSeeked = false;
      //   pop();
      // } else {
      //   d.setPower(0, dist/1000, 0, 0);
      //   globalx += d.getClickslf();
      //   d.resetEncoderlf();
      // }
    }
    if(states[count] == "collect") {
      if (collect.setPos("collect")) { pop(); }
    }
    if(states[count] == "out") {
      if (collect.setPos("up")) { pop(); }
    }
    if(states[count] == "lift"){
      if (isArgs[count]) {
        lift.motor.setPower(-0.8);
        sleep(2000);
        lift.motor.setPower(0);
        sleep(500);
        lift.motor.setPower(0.7);
        sleep(2000);
        pop();
        isArgs[count] = false;
      } else {
        pop();
      }
    }
    // if(states[count] == "sleep") {
    //   if(isArgs[count]) {
    //     long slep = args[count];
    //     sleep(slep);
    //     isArgs[count] = false;
    //     pop();
    //   } else {
    //     pop();
    //   }
    // }
  }


  public void lift(double clicks){
      liftstartclicks = lift.getClicks(); // where the encoder starts
      liftgoalclicks = liftstartclicks + clicks; // how far to go
  }
  public void lift(){
    double current = lift.getClicks();
    if(current > liftgoalclicks - 40 && current < liftgoalclicks + 40) {
      lift.motor.setPower(0);
      pop();
    } else if(current < liftgoalclicks) {
      lift.motor.setPower(0.7);
    } else if(current > liftgoalclicks) {
      lift.motor.setPower(-0.8);
    }
  }

  public void turn() {
    // this.turning = true;
    // theta = getAngle();
    // d.setPower(0, 0, (dtheta - theta) / (Math.abs(dtheta - theta)) , 0.6);
    // if(Math.abs(theta - dtheta) < 2) { //if diff is less than 2 degrees
    //   this.turning = false;
    //   globala = getAngle();
    //   pop();
    // }
    globala = 180;
    pop();
  }
  public void turn(double degrees){
    dtheta = theta + degrees;
  }

  public void correct() {
    double current = getAngle();
    double power =  (globala - current) / (Math.abs(globala - current));
    if (globala - current > 1) {
      d.setPower(d.getLy(), d.getLx(), power , d.getTurbo());
    }
    if (globala - current < -1) {
      d.setPower(d.getLy(), d.getLx(), power/2 , d.getTurbo());
    }
  }

  public void setGlobaly () {
    if (globala > 170 && globala < 190) {
      globaly -= d.getClickslf();
    } else {
      globaly += d.getClickslf();
    }
    d.resetEncoderlf();
  }

  public void setGlobalx () {
    if (globala > 170 && globala < 190) {
      globalx -= d.getClickslf();
    } else {
      globalx += d.getClickslf();
    }
    d.resetEncoderlf();
  }

  public void forwardTo(double y){ //init forward function
    // double dist = y-globaly;
    if (globala > 170 && globala < 190) { //turned around
      forward(-y);
    } else {
      forward(y);
    }
  }

  public void forward(double clicks){
    d.resetEncoderlf();
    goalclicks = clicks; // how far to go
    // relativey = globaly;
  }
  public void forward(){
    double dist = Math.abs(goalclicks) - Math.abs(relativey);
    double p = Math.abs(dist)/120;
    if(relativey > goalclicks - 25 && relativey < goalclicks + 25) {
      setGlobaly();
      pop();
    } else if(relativey < goalclicks) {
      d.setPower(1, 0, 0, 0.6*p);
      relativey += d.getClickslf();
      setGlobaly();
    } else if(relativey > goalclicks) {
      d.setPower(-1, 0, 0, 0.6*p);
      relativey += d.getClickslf();
      setGlobaly();
    }
  }

  public void strafeTo(double x){ //init forward function
    if (globala > 170 && globala < 190) { //turned around
      strafe(-x);
    } else {
      strafe(x);
    }
  }
  public void strafe(double clicks) {
    d.resetEncoderlf();
    goalclicks = clicks; // how far to go
  }
  public void strafe() {
    double dist = Math.abs(goalclicks) - Math.abs(relativey);
    double p = Math.abs(dist)/120;
    if(relativex > goalclicks - 75 && relativex < goalclicks + 75) {
      setGlobalx();
      pop();
    } else if(relativex < goalclicks) {
      d.setPower(0, -1, 0, 0.6*p);
      relativex += d.getClickslf();
      setGlobalx();
    } else if(relativex > goalclicks) {
      d.setPower(0, 1, 0, 0.6*p);
      relativex += d.getClickslf();
      setGlobalx();
    }
  }


  public double find() {
    double[] result = f.findSkystoneAngle();
    double angle = result[0];
    return angle;
  }

  public double getAngle() {
    //this function taken from somewhere else

    // We experimentally determined the Z axis is the axis we want to use for heading angle.
    // We have to process the angle because the imu works in euler angles so the Z axis is
    // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
    // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

    if (deltaAngle < -180)
        deltaAngle += 360;
    else if (deltaAngle > 180)
        deltaAngle -= 360;

    globalAngle += deltaAngle;

    lastAngles = angles;

    return globalAngle;
  }

  public void runOpMode() {

  }

}
