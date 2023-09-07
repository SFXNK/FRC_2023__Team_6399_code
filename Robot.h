// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
private:
  float FWD, STR, RCW, A, B, C, D, ws[5], wa[5], na[5];
  float L=62, W=62, R=sqrt(L*L+W*W);
  float pi=3.1415926;
  double targetpos[5],lastpos[5];
  float la[5];
  long aconst[5];
  float maxx;
  float maxp;
  float maxs;
  int tickrate;
  bool mdir[5];
  double ang=0,accx,accy,accz;
};
