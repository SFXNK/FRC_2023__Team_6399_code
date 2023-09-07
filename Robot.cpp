#include "Robot.h"
#include <frc/ADXL345_I2C.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <cameraserver/CameraServer.h>
#include <iostream>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <cmath>
#include <math.h>
#define ADDR 0x52
frc::ADXL345_I2C my_gyro{frc::I2C::Port::kOnboard,frc::Accelerometer::kRange_16G, 0X53};

TalonFX MAA(11);
TalonFX MAS(12);
TalonFX MBA(13);
TalonFX MBS(14);
TalonFX MCA(15);
TalonFX MCS(16);
TalonFX MDA(17);
TalonFX MDS(18);
TalonFX SWA(1);
TalonFX SWB(2);
TalonFX MID(3);
TalonFX CLBL(6);
TalonFX CLBR(7);
frc::Timer tm;
frc::DutyCycleEncoder eca{0};
frc::DutyCycleEncoder ecb{1};
frc::DutyCycleEncoder ecc{2};
frc::DutyCycleEncoder ecd{3};
double encoderOffset[4]={0.835,0.548,0.011,0.545};
double mcv=100000,ma=25000;
 /*
  2 1
  3 4

  B A
  C D
  */
frc::XboxController xboxx{0};
double voltadj(double base){
  frc::SmartDashboard::PutNumber("Shooter Power",base);
  return base;
}
double mod180(double a){
    if(a>180.00001){
        a-=(int)(a+180)/360*360;
    }
    if(a<-180.00001){
        a-=(int)(a-180)/360*360;
    }
    return a;
}
double modd(double a){
  while(a>1){
    a-=2;
  }
  while(a<-1){
    a+=2;
  }
  return a;
}
double absminn(double a,double b,double c){
  return abs(a)<abs(b)?(abs(a)<abs(c)?a:c):(abs(b)<abs(c)?b:c);
}

void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
  frc::CameraServer::StartAutomaticCapture();
  tickrate=1000;
  double kf=0.0;
  double kp=1.0;
  double ki=0.001;
  double kd=10.0;
  maxp=1.0;
  maxs=0.65;
  //MAA.ConfigFactoryDefault();
  //MBA.ConfigFactoryDefault();
  //MCA.ConfigFactoryDefault();
  //MDA.ConfigFactoryDefault();
  eca.SetPositionOffset(0.835);
  ecb.SetPositionOffset(0.548);
  ecc.SetPositionOffset(0.011);
  ecd.SetPositionOffset(0.545);
  eca.SetDistancePerRotation(2.0);
  ecb.SetDistancePerRotation(2.0);
  ecc.SetDistancePerRotation(2.0);
  ecd.SetDistancePerRotation(2.0);
  MAA.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
  MBA.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
  MCA.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
  MDA.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
  //ITM.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
  MAA.SetSelectedSensorPosition(0, 0, 10);
  MBA.SetSelectedSensorPosition(0, 0, 10);
  MCA.SetSelectedSensorPosition(0, 0, 10);
  MDA.SetSelectedSensorPosition(0, 0, 10);
 // ITM.SetSelectedSensorPosition(0, 0, 10);
  MAA.SelectProfileSlot(0, 0);
  MAA.Config_kF(0, 0.3);
  MAA.Config_kP(0, kp);
  MAA.Config_kI(0, ki);
  MAA.Config_kD(0, kd);
  MAA.ConfigMotionCruiseVelocity(mcv);
  MAA.ConfigMotionAcceleration(ma);
  MBA.SelectProfileSlot(0, 0);
  MBA.Config_kF(0, 0.3);
  MBA.Config_kP(0, kp);
  MBA.Config_kI(0, ki);
  MBA.Config_kD(0, kd);
  MBA.ConfigMotionCruiseVelocity(mcv);
  MBA.ConfigMotionAcceleration(ma);  
  MCA.SelectProfileSlot(0, 0);
  MCA.Config_kF(0, 0.3);
  MCA.Config_kP(0, kp);
  MCA.Config_kI(0, ki);
  MCA.Config_kD(0, kd);
  MCA.ConfigMotionCruiseVelocity(mcv);
  MCA.ConfigMotionAcceleration(ma);  
  MDA.SelectProfileSlot(0, 0);
  MDA.Config_kF(0, 0.3);
  MDA.Config_kP(0, kp);
  MDA.Config_kI(0, ki);
  MDA.Config_kD(0, kd);
  MDA.ConfigMotionCruiseVelocity(mcv);
  MDA.ConfigMotionAcceleration(ma);
  //ITM.SelectProfileSlot(0, 0);
  //ITM.Config_kF(0, 0.3);
  //ITM.Config_kP(0, kp);
  //ITM.Config_kI(0, ki);
  //ITM.Config_kD(0, kd);
  //ITM.ConfigMotionCruiseVelocity(mcv);
  //ITM.ConfigMotionAcceleration(ma);
}
void Robot::RobotPeriodic() {
  /*
  frc::SmartDashboard::PutNumber("Angle 1",na[1]);
  frc::SmartDashboard::PutNumber("Angle 2",na[2]);
  frc::SmartDashboard::PutNumber("Angle 3",na[3]);
  frc::SmartDashboard::PutNumber("Angle 4",na[4]);

  frc::SmartDashboard::PutNumber("P1",ws[1]);
  frc::SmartDashboard::PutNumber("P2",ws[2]);
  frc::SmartDashboard::PutNumber("P3",ws[3]);
  frc::SmartDashboard::PutNumber("P4",ws[4]);
  */
  
  frc::SmartDashboard::PutNumber("encoder1",eca.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("encoder2",ecb.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("encoder3",ecc.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("encoder4",ecd.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("encoder1P",eca.GetDistance());
  frc::SmartDashboard::PutNumber("encoder2P",ecb.GetDistance());
  frc::SmartDashboard::PutNumber("encoder3P",ecc.GetDistance());
  frc::SmartDashboard::PutNumber("encoder4P",ecd.GetDistance());

  //double volt=(SWA.GetOutputCurrent()+SWB.GetOutputCurrent())/2;
  //frc::SmartDashboard::PutNumber("Shooter Current",volt);
}

void Robot::AutonomousInit() {
  tm.Reset();
  tm.Start();
}
void Robot::AutonomousPeriodic() {
  double sht=0.75;
  while((double)tm.Get()<7.0){
    MAS.Set(ControlMode::PercentOutput,-0.075);
    MBS.Set(ControlMode::PercentOutput,-0.075);
    MCS.Set(ControlMode::PercentOutput,-0.075);
    MDS.Set(ControlMode::PercentOutput,-0.075);
    MAA.Set(ControlMode::Disabled,0);
    MBA.Set(ControlMode::Disabled,0);
    MCA.Set(ControlMode::Disabled,0);
    MDA.Set(ControlMode::Disabled,0);
  }
  while((double)tm.Get()<10.0){
    SWA.Set(ControlMode::PercentOutput,-voltadj(sht)*maxs);
    SWB.Set(ControlMode::PercentOutput,voltadj(sht)*maxs);
    MAS.Set(ControlMode::PercentOutput,-0.075);
    MBS.Set(ControlMode::PercentOutput,-0.075);
    MCS.Set(ControlMode::PercentOutput,-0.075);
    MDS.Set(ControlMode::PercentOutput,-0.075);
    MAA.Set(ControlMode::Disabled,0);
    MBA.Set(ControlMode::Disabled,0);
    MCA.Set(ControlMode::Disabled,0);
    MDA.Set(ControlMode::Disabled,0);
  }
  while((double)tm.Get()<14.0){
    MAS.Set(ControlMode::PercentOutput,-0.06);
    MBS.Set(ControlMode::PercentOutput,-0.06);
    MCS.Set(ControlMode::PercentOutput,-0.06);
    MDS.Set(ControlMode::PercentOutput,-0.06);
    MAA.Set(ControlMode::Disabled,0);
    MBA.Set(ControlMode::Disabled,0);
    MCA.Set(ControlMode::Disabled,0);
    MDA.Set(ControlMode::Disabled,0);
    SWA.Set(ControlMode::PercentOutput,-voltadj(sht)*maxs);
    SWB.Set(ControlMode::PercentOutput,voltadj(sht)*maxs);
    MID.Set(ControlMode::PercentOutput,0.35);
  }
  while((double)tm.Get()<17.0){
    MAS.Set(ControlMode::PercentOutput,0);
    MBS.Set(ControlMode::PercentOutput,0);
    MCS.Set(ControlMode::PercentOutput,0);
    MDS.Set(ControlMode::PercentOutput,0);
    MAA.Set(ControlMode::Disabled,0);
    MBA.Set(ControlMode::Disabled,0);
    MCA.Set(ControlMode::Disabled,0);
    MDA.Set(ControlMode::Disabled,0);
    SWA.Set(ControlMode::PercentOutput,0);
    SWB.Set(ControlMode::PercentOutput,0);
    MID.Set(ControlMode::PercentOutput,0);
  }

}

void Robot::TeleopInit() {
  tm.Reset();
  tm.Start();
  aconst[1]=0;
  aconst[2]=0;
  aconst[3]=0;
  aconst[4]=0;
  for(int i=1;i<=4;i++){
    mdir[i]=true;
  }
}
void Robot::TeleopPeriodic() {
  double ts=(double)tm.Get();
  FWD=-xboxx.GetLeftY();
  STR=xboxx.GetLeftX();
  RCW=xboxx.GetRightX();

  //frc::SmartDashboard::PutNumber("FWD",FWD);
  //frc::SmartDashboard::PutNumber("STR",STR);
  //frc::SmartDashboard::PutNumber("RCW",RCW);

  if((FWD>0?FWD:-FWD)<0.07)FWD=0;
  if((STR>0?STR:-STR)<0.07)STR=0; 
  if((RCW>0?RCW:-RCW)<0.07)RCW=0;

  A=STR-RCW*(L/R);
  B=STR+RCW*(L/R);
  C=FWD-RCW*(W/R);
  D=FWD+RCW*(W/R); 

  ws[1] = sqrt(B*B+C*C); 
  wa[1] = atan2(B,C)*180/pi; 
  ws[2] = sqrt(B*B+D*D); 
  wa[2] = atan2(B,D)*180/pi; 
  ws[3] = sqrt(A*A+D*D); 
  wa[3] = atan2(A,D)*180/pi; 
  ws[4] = sqrt(A*A+C*C); 
  wa[4] = atan2(A,C)*180/pi;

  //frc::SmartDashboard::PutNumber("AccX",accx);
  //frc::SmartDashboard::PutNumber("AccY",accy);
  //frc::SmartDashboard::PutNumber("AccZ",accz);
  //frc::SmartDashboard::PutNumber("Ang",ang);
  maxx=1;
  for(int i=1;i<=4;i++){
    if(ws[i]>maxx)
      maxx=ws[i];
  }
  if(maxx>1)
    for(int i=1;i<=4;i++){
      ws[i]/=maxx;
    }
  double sht=xboxx.GetRawAxis(3);
  sht=sht<0.05?0:sht;
  double itk=xboxx.GetRawAxis(2);
  itk=itk<0.05?0:itk;
  //ITC.Set(ControlMode::PercentOutput,-itk*0.8);
  if(xboxx.GetRawButton(8)){
    SWA.Set(ControlMode::PercentOutput,0.8*maxs);
    SWB.Set(ControlMode::PercentOutput,-0.8*maxs);
  }
  else{
    SWA.Set(ControlMode::PercentOutput,-voltadj(sht*maxs));
    SWB.Set(ControlMode::PercentOutput,voltadj(sht*maxs));
  }
  if(xboxx.GetPOV()==0){
    CLBL.Set(ControlMode::PercentOutput,1.0);
    CLBR.Set(ControlMode::PercentOutput,-1.0);
  }
  else if(xboxx.GetPOV()==180){
    CLBL.Set(ControlMode::PercentOutput,-0.8);
    CLBR.Set(ControlMode::PercentOutput,0.8);
  }
  else if(xboxx.GetPOV()==270){
    CLBR.Set(ControlMode::PercentOutput,1.0);
  }
  else if(xboxx.GetPOV()==90){
    CLBL.Set(ControlMode::PercentOutput,-1.0);
  }
  else{
    CLBL.Set(ControlMode::Disabled,0);
    CLBR.Set(ControlMode::Disabled,0);
  }
  if(xboxx.GetRawButton(7)){
    MAA.SetSelectedSensorPosition(0, 0, 10);
    MBA.SetSelectedSensorPosition(0, 0, 10);
    MCA.SetSelectedSensorPosition(0, 0, 10);
    MDA.SetSelectedSensorPosition(0, 0, 10);
  }

  if(xboxx.GetRawButton(2)){
    MID.Set(ControlMode::PercentOutput,0.3);
  }
  else{
    MID.Set(ControlMode::PercentOutput,0);
  }

  if(xboxx.GetRawButton(3)){
    maxp=0.25;
  }
  else if(xboxx.GetRawButton(4)){
    maxp=1.0;
  }
  if(xboxx.GetRawButton(5)){
    //ITM.Set(ControlMode::PercentOutput,-0.07);
  }
  else if(xboxx.GetRawButton(6)){
    //ITM.Set(ControlMode::PercentOutput,0.15);
  }
  else{
    //ITM.Set(ControlMode::PercentOutput,0);
  }

  if(FWD==0 && STR==0 && RCW==0){
    MAS.Set(ControlMode::Disabled,1);
    MBS.Set(ControlMode::Disabled,1);
    MCS.Set(ControlMode::Disabled,1);
    MDS.Set(ControlMode::Disabled,1);
    if(xboxx.GetRawButton(1)){
      na[1]=(mdir[1]==true)?mod180(MAA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MAA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
      na[2]=(mdir[2]==true)?mod180(MBA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MBA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
      na[3]=(mdir[3]==true)?mod180(MCA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MCA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
      na[4]=(mdir[4]==true)?mod180(MDA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MDA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
      for(int i=1;i<=4;i++){
        if(mdir[i]!=true){
          mdir[i]=true;
          na[i]=mod180(na[i]+180);
        }
        targetpos[i]+=(-na[i])/360*2048*21.42857;
      }
      MAA.Set(ControlMode::MotionMagic,targetpos[1]);
      MBA.Set(ControlMode::MotionMagic,targetpos[2]);
      MCA.Set(ControlMode::MotionMagic,targetpos[3]);
      MDA.Set(ControlMode::MotionMagic,targetpos[4]);
      while(xboxx.GetRawButton(1)){}
      //frc::SmartDashboard::PutNumber("motion direction 1",mdir[1]);
      //frc::SmartDashboard::PutNumber("motion direction 2",mdir[2]);
      //frc::SmartDashboard::PutNumber("motion direction 3",mdir[3]);
      //frc::SmartDashboard::PutNumber("motion direction 4",mdir[4]);
    }
    else{
    MAA.Set(ControlMode::Disabled,0);
    MBA.Set(ControlMode::Disabled,0);
    MCA.Set(ControlMode::Disabled,0);
    MDA.Set(ControlMode::Disabled,0);
    }
    return;
  }

  na[1]=(mdir[1]==true)?mod180(MAA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MAA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
  na[2]=(mdir[2]==true)?mod180(MBA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MBA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
  na[3]=(mdir[3]==true)?mod180(MCA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MCA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
  na[4]=(mdir[4]==true)?mod180(MDA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857):mod180(MDA.GetSelectedSensorPosition(0)*360.0/2048.0/21.42857+180);
  
  targetpos[1]=MAA.GetSelectedSensorPosition(0);
  targetpos[2]=MBA.GetSelectedSensorPosition(0);
  targetpos[3]=MCA.GetSelectedSensorPosition(0);
  targetpos[4]=MDA.GetSelectedSensorPosition(0);
  
  for(int i=1;i<=4;i++){
    
    if((wa[i]-na[i]>90 && wa[i]-na[i]<270) || (wa[i]-na[i]<-90 && wa[i]-na[i]>-270)){
        mdir[i]=(mdir[i]==true)?false:true;
        na[i]=mod180(na[i]+180);
    }
    
    targetpos[i]+=absminn(wa[i]-na[i],-360+(wa[i]-na[i]),360+(wa[i]-na[i]))/360*2048*21.42857;
  }
  
  MAA.Set(ControlMode::MotionMagic,targetpos[1]);
  MBA.Set(ControlMode::MotionMagic,targetpos[2]);
  MCA.Set(ControlMode::MotionMagic,targetpos[3]);
  MDA.Set(ControlMode::MotionMagic,targetpos[4]);
    
  MAS.Set(ControlMode::PercentOutput,mdir[1]==true?ws[1]*maxp:-ws[1]*maxp);
  MBS.Set(ControlMode::PercentOutput,mdir[2]==true?ws[2]*maxp:-ws[2]*maxp);
  MCS.Set(ControlMode::PercentOutput,mdir[3]==true?ws[3]*maxp:-ws[3]*maxp);
  MDS.Set(ControlMode::PercentOutput,mdir[4]==true?ws[4]*maxp:-ws[4]*maxp);

  //frc::SmartDashboard::PutNumber("Target Angle 1",wa[1]);
  //frc::SmartDashboard::PutNumber("Target Angle 2",wa[2]);
  //frc::SmartDashboard::PutNumber("Target Angle 3",wa[3]);
  //frc::SmartDashboard::PutNumber("Target Angle 4",wa[4]);
  
  //frc::SmartDashboard::PutNumber("motion direction 1",mdir[1]?1:-1);
  //frc::SmartDashboard::PutNumber("motion direction 2",mdir[2]?1:-1);
  //frc::SmartDashboard::PutNumber("motion direction 3",mdir[3]?1:-1);
  //frc::SmartDashboard::PutNumber("motion direction 4",mdir[4]?1:-1);

  //frc::SmartDashboard::PutNumber("TIME",(double)tm.Get()-ts);
  while((double)tm.Get()<ts+1.0/tickrate){}
}

void Robot::DisabledInit() {

}
void Robot::DisabledPeriodic() {}
 

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif