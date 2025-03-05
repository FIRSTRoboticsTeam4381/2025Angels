// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase {

    public LEDS() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
       AddressableLED m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
       AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

         // Create the buffer
AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);

// Create the view for the section of the strip on the left side of the robot.
// This section spans LEDs from index 0 through index 59, inclusive.
AddressableLEDBufferView m_left = m_buffer.createView(0, 59);

// The section of the strip on the right side of the robot.
// This section spans LEDs from index 60 through index 119, inclusive.
// This view is reversed to cancel out the serpentine arrangement of the
// physical LED strip on the robot.
AddressableLEDBufferView m_right = m_buffer.createView(60, 119).reversed();



public Red(){
    // Create an LED pattern that sets the entire strip to solid red
LEDPattern red = LEDPattern.solid(Color.kRed);

// Apply the LED pattern to the data buffer
red.applyTo(m_ledBuffer);

// Write the data to the LED strip
m_led.setData(m_ledBuffer);
    }
}
}
