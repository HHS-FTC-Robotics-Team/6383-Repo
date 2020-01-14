// package org.firstinspires.ftc.teamcode;

// import android.app.Activity;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import android.view.View;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.ColorSensor;

// public class Color extends LinearOpMode {
  
//   ColorSensor colorSensor;    // Hardware Device Object


//   public Color(ColorSensor col) {
//     colorSensor = col;
//     // hsvValues is an array that will hold the hue, saturation, and value information.
//     float hsvValues[] = {0F,0F,0F};
//     colorSensor.enableLed(true);
//     Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

//   }

 

//   public double getClicks() {
//     double cl = motor.getCurrentPosition();
//     return cl;
//   }

//   public double getPower() {
//     double p = motor.getPower();
//     return p;
//   }

//   public void runOpMode() {
//   }
// }

// //     // hsvValues is an array that will hold the hue, saturation, and value information.
// //     float hsvValues[] = {0F,0F,0F};

// //     // values is a reference to the hsvValues array.
// //     final float values[] = hsvValues;

// //     // get a reference to the RelativeLayout so we can change the background
// //     // color of the Robot Controller app to match the hue detected by the RGB sensor.
// //     int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
// //     final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

// //     // bPrevState and bCurrState represent the previous and current state of the button.
// //     boolean bPrevState = false;
// //     boolean bCurrState = false;

// //     // bLedOn represents the state of the LED.
// //     boolean bLedOn = true;

// //     // get a reference to our ColorSensor object.
// //     colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");

// //     // Set the LED in the beginning
// //     colorSensor.enableLed(bLedOn);

// //     // wait for the start button to be pressed.
// //     waitForStart();

// //     // while the op mode is active, loop and read the RGB data.
// //     // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
// //     while (opModeIsActive()) {

// //       // check the status of the x button on either gamepad.
// //       bCurrState = gamepad1.x;

// //       // check for button state transitions.
// //       if (bCurrState && (bCurrState != bPrevState))  {

// //         // button is transitioning to a pressed state. So Toggle LED
// //         bLedOn = !bLedOn;
// //         colorSensor.enableLed(bLedOn);
// //       }

// //       // update previous state variable.
// //       bPrevState = bCurrState;

// //       // convert the RGB values to HSV values.
// //       // Carlo note: HSV is like rgb but it measures intensity, see the 'hue' value in the telemetry thing below
// //       Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

// //       // send the info back to driver station using telemetry function.
// //       telemetry.addData("LED", bLedOn ? "On" : "Off");
// //       telemetry.addData("Clear", colorSensor.alpha());
// //       telemetry.addData("Red  ", colorSensor.red());
// //       telemetry.addData("Green", colorSensor.green());
// //       telemetry.addData("Blue ", colorSensor.blue());
// //       telemetry.addData("Hue", hsvValues[0]);

// //       // change the background color to match the color detected by the RGB sensor.
// //       // pass a reference to the hue, saturation, and value array as an argument
// //       // to the HSVToColor method.
// //       relativeLayout.post(new Runnable() { //Carlo note: here it takes the HSV and turns it to Color, whatever color means
// //         public void run() {                //and uses Color to set the background on the phone
// //           relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
// //         }
// //       });

// //       telemetry.update();
// //     }

// //     // Set the panel back to the default color
// //     relativeLayout.post(new Runnable() {
// //       public void run() {
// //         relativeLayout.setBackgroundColor(Color.WHITE);
// //       }
// //     });
// //   }
// // }

