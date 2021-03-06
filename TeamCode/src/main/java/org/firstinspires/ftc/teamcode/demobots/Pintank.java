package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;

@TeleOp(name="Pintank",group="teleop")
public class Pintank extends LinearOpMode
{

    AnalogInput irSensor;
    AnalogOutput irBeacon;

    @Override
    public void runOpMode() throws InterruptedException
    {

        //irBeacon=hardwareMap.analogOutput.get("beacon");
        irSensor=hardwareMap.analogInput.get("irSensor");
        waitForStart();
        while(opModeIsActive())
        {

            telemetry.addData("ir sensor",irSensor.getVoltage());
            telemetry.update();

        }

    }
}
