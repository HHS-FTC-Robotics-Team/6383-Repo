package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.SciLift;
import org.firstinspires.ftc.teamcode.Arm;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Chomp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="6383 Teleop", group="Linear Opmode")

public class Teleop6383 extends LinearOpMode {

    private Drive d;
    private SciLift lift;
    private Chomp chomp; //now controls one servo, the claw
    private Arm arm;

    @Override
    public void runOpMode() {

      d = new Drive(
          hardwareMap.get(DcMotor.class, "motorrb"),
          hardwareMap.get(DcMotor.class, "motorrf"),
          hardwareMap.get(DcMotor.class, "motorlf"),
          hardwareMap.get(DcMotor.class, "motorlb")
      );
      lift = new SciLift(
        hardwareMap.get(DcMotor.class, "liftmotor")
      );
      arm = new Arm(
        hardwareMap.get(DcMotor.class, "armmotor")
      );
      chomp = new Chomp(
        hardwareMap.get(Servo.class, "claw")
      );

      waitForStart();
      while (opModeIsActive()) {

        d.setPower(
          -gamepad1.left_stick_y,
          -gamepad1.left_stick_x,
          gamepad1.right_stick_x,
          0.85 - (gamepad1.right_trigger * 0.8)
        );

        if (gamepad1.b) {
          // chomp.in();
          chomp.setPos("up");
        }
        if (gamepad1.a) {
          // chomp.out();
          chomp.setPos("collect");
        }


        if (gamepad1.left_bumper) {
          // if (arm.getClicks() < 20) {
            arm.retract();
          // }
        } else if (gamepad1.left_trigger > 0){
          // if (arm.getClicks() < 300) {
            arm.extend(gamepad1.left_trigger);
          // }
        } else {
          arm.rest();
        }

        if (gamepad1.x) {
            lift.down();
        } else if (gamepad1.y){
            lift.up();
        } else {
            lift.rest();
        }

        telemetry.addData("Status", "Run Time: ");
        telemetry.addData("Lift Power", lift.getPower());
        telemetry.addData("clawpos", chomp.position);
        telemetry.addData("armpos", arm.getClicks());
        telemetry.addData("Ly", -gamepad1.left_stick_y);
        telemetry.addData("Lx", -gamepad1.left_stick_x);
        telemetry.addData("Rx", gamepad1.right_stick_x);
        telemetry.addData("clicksdrive", d.getClickslf());
        telemetry.addData("lf", d.getPowerlf());
        telemetry.addData("lb", d.getPowerlb());
        telemetry.addData("rf", d.getPowerrf());
        telemetry.addData("rb", d.getPowerrb());
        telemetry.update();

      }
    }
}
