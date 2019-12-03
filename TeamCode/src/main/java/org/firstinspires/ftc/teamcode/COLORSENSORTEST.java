package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
@Disabled
public class COLORSENSORTEST extends LinearOpMode {

    private ColorSensor cdsensor;

    @Override
    public void runOpMode(){
        //color distance sensor, 'imu' in hardwaremap
        cdsensor = hardwareMap.get(ColorSensor.class, "imu");




        telemetry.addData("Status:", "Running");
        telemetry.update();


        waitForStart();

        while(opModeIsActive()){

            cdsensor.enableLed(true);

            //YELLOW: RGB RED-255, GREEN 255, BLUE 0 usually above 150 for all vals
            //BLACK: 0 0 0; usually below 100 for all vals

            telemetry.addData("IS SKYSTONE? ", isSkystone(cdsensor.red(), cdsensor.green(), cdsensor.blue()));


            telemetry.addData("-------", "-----");

            telemetry.addData("color RED: ", cdsensor.red());
            telemetry.addData("color GREEN: ", cdsensor.green());
            telemetry.addData("color BLUE: ", cdsensor.blue());

            telemetry.addData("-------", "-----");

            telemetry.addData("color sensor rgb: ", cdsensor.argb());
            telemetry.addData("color sensor light: ", cdsensor.alpha());


            telemetry.update();
        }


    }
    public boolean isSkystone(int red, int green, int blue){
        return !(red > 300 && green > 300 && blue > 150);
    }



}
