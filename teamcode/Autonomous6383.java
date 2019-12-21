/*
Copyright 2019 FIRST Tech Challenge Team 6383

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Gpsbrain;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Chomp;
import org.firstinspires.ftc.teamcode.Find;


@Autonomous

public class Autonomous6383 extends LinearOpMode {

    private Drive d;
    private Chomp c;
    private BNO055IMU imu;
    private Gpsbrain gps;
    private Find f;
    private SciLift lift;

    @Override
    public void runOpMode() {

        d = new Drive(
          hardwareMap.get(DcMotor.class, "motorrb"),
          hardwareMap.get(DcMotor.class, "motorrf"),
          hardwareMap.get(DcMotor.class, "motorlf"),
          hardwareMap.get(DcMotor.class, "motorlb")
        );

        c = new Chomp(
          hardwareMap.get(Servo.class, "claw")
        );
        
        lift = new SciLift(
            hardwareMap.get(DcMotor.class, "liftmotor")
        );
        
        f = new Find();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        gps = new Gpsbrain(d, imu, c, f, lift);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //For testing, push operation here:
        // gps.forward();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gps.update(); //all the autonomous stuff
            if (!gps.turning) { gps.correct(); } //keeping the robot straight

            telemetry.addData("State", gps.states[gps.count]);
            telemetry.addData("Count", gps.count);
            telemetry.addData("Angle", gps.seekAngle);
            telemetry.addData("clicks", gps.d.getClickslf());
            telemetry.addData("globalx", gps.globalx);
            telemetry.addData("globaly", gps.globaly);
            telemetry.addData("globala", gps.globala);
            telemetry.addData("goalclicks", gps.goalclicks);
            telemetry.addData("globaly", gps.relativey);
            telemetry.addData("servo", gps.collect.servo.getPosition());
            telemetry.update();

        }
    }
}
