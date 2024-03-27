// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class LedsIOReal implements LedsIO {
  private static LedsIOReal instance;

  public static LedsIOReal getInstance() {
    if (instance == null) {
      instance = new LedsIOReal(); // Q1
    }
    return instance;
  }

  private static final int length = 144;

  public final AddressableLED m_Led;
  public final AddressableLEDBuffer m_LedBuffer;

  /** Creates a new LedsIOReal. */
  public LedsIOReal() {
    m_Led = new AddressableLED(1);
    m_LedBuffer = new AddressableLEDBuffer(length);
    m_Led.setLength(length);
    m_Led.setData(m_LedBuffer);
    m_Led.start();
  }

  @Override
  public void setColor(int rValue, int gValue, int bValue) {
    for (int i = 0; i < m_LedBuffer.getLength(); i++){
      m_LedBuffer.setRGB(i, rValue, gValue, bValue);
    }
    
    m_Led.setData(m_LedBuffer);
  } 
/* 
  public void wave(Color c1, Color c2, double cycleLength, double duration){
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPeriod = (2.0 * Math.PI) / cycleLength;

    x += xDiffPeriod;

    double ratio = (Math.pow(Math.sin(x), 0.4) + 1.0) / 2.0;
    if(Double.isNaN(ratio)){
      ratio = (-Math.pow(Math.sin(x), 0.4) + 1.0) / 2.0 ;
    }
    if(Double.isNaN(ratio)){
      ratio = 0.5;
    }
    double red = (c1)

  }*/
  
  @Override
  public void allianceIdleColor() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      setColor(255, 0, 0);
    } else {
      setColor(0, 0, 255);
    }
  }
  
}