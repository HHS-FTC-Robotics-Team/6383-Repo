package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


public class Chomp {
    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo;
    Rev2mDistanceSensor dist;
    // double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double position = 0.1;
    boolean rampUp = true;

    public Chomp(Servo s, Rev2mDistanceSensor d) {
      servo = s;
      servo.setPosition(position);
      dist = d;
    }

    public void setPos(double goal) {
    double error = 0.05;
    double increment = 0.07;
    double pos1 = servo.getPosition();
    if (pos1 > goal) {
      pos1 -= increment;
    } else if (pos1 < goal){
      pos1 += increment;
    } else if (pos1 < goal - error && pos1 > goal + error) {
      pos1 = goal;
    }
      servo.setPosition(pos1);
    }

    public boolean setPos(String specificGoal) {
      double goal = 0;
      if (specificGoal == "up") {
        goal = 0.0;
      } else if (specificGoal == "down") {
        goal = 0.7;
      } else if (specificGoal == "collect") {
        goal = 0.60;
      }
      double error = 0.015;
      double increment = 0.01;
      double pos1 = servo.getPosition();
      if (pos1 > goal - error && pos1 < goal + error) {
        pos1 = goal;
        return true;
      } else if (pos1 > goal) {
        pos1 -= increment;
      } else if (pos1 < goal){
        pos1 += increment;
      }
      servo.setPosition(pos1);
      return false;
    }
    
    public boolean checkPos(String specificGoal) {
      double goal = 0;
      if (specificGoal == "up") {
        goal = 0.0;
      } else if (specificGoal == "down") {
        goal = 0.7;
      } else if (specificGoal == "collect") {
        goal = 0.60;
      }
      double error = 0.015;
      double increment = 0.01;
      double pos1 = servo.getPosition();
      if (pos1 > goal - error && pos1 < goal + error) {
        pos1 = goal;
        return true;
      } else {
        return false;
      }
    }

    public void out() {
        if (position <= 0.99) {
            position += INCREMENT ;
        } else {
            position = 0.99;
        }
        servo.setPosition(position);
    }

    public void out(double amt) {
      position += amt;
      if (position > 0.99) {
        position = 0.99;
      }
      servo.setPosition(position);
    }

    public void in() {
      if (position >= 0.01) {
          position -= INCREMENT ;
      } else {
          position = 0.01;
      }
      servo.setPosition(position);
    }

    public void in(double amt) {
      position -= amt;
      if (position < 0.01) {
        position = 0.01;
      }
      servo.setPosition(position);
    }
    
    public double getDist() {
      double distance = dist.getDistance(DistanceUnit.CM);
      return distance;
    }

    public void runOpMode() {

    }
}
