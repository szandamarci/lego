import hub,hub_runtime,utime
import json,os
class Mot(object):
  def __init__(self,port):
    self.Port=port
    self.Zero()
    self.PosAct=0.0
  def Zero(self):
    self.SetPointIn=0.0
    self.LastPoint=0.0
    self.Integral=0.0
    self.CurrentError=0.0
    self.LastError=0.0
    self.AxOffTime=0.0
    self.PwmAct=0.0
    self.Max=100.0
  def Exe(self,motorsOn,tickDiff):
    SetPoint=self.SetPointIn
    MotorOn=(self.LastPoint!=SetPoint)
    self.LastPoint=SetPoint
    self.PosAct=self.Port.device.get()[0]
    self.CurrentError=SetPoint-self.PosAct
    self.Integral=self.Integral+self.CurrentError*tickDiff
    self.Integral=Clamp(self.Integral,10000.0)
    Derivative=(self.CurrentError-self.LastError)/tickDiff
    self.LastError=self.CurrentError
    Kp=0.2
    Ki=0.0009
    Kd=0.0
    Output=Kp*(self.CurrentError/tickDiff*10)+Kd*Derivative+Ki*self.Integral
    if (MotorOn==False) and ((abs(self.CurrentError)<5) or (self.AxOffTime>2000)):
      Output=0
    if MotorOn:
      self.AxOffTime=0
    else:
      self.AxOffTime=self.AxOffTime+tickDiff
    if motorsOn:
      self.SetPwm(Clamp(Output,self.Max))
    return self.PosAct
  def SetPwm(self,Power):
    self.PwmAct=Power
    self.Port.pwm(int(Power))
  def MovePos(self,Pos,Power):
    self.Port.motor.run_to_position(int(Pos),Power)
  def MovePosWait(self,Pos,Power):
    posTmp=100000.0
    self.Port.motor.run_to_position(int(Pos),Power)
    while (self.GetPos()!=posTmp):
      posTmp=self.GetPos()
      utime.sleep(0.500)
    self.PosAct=self.Port.device.get()[0]
  def SetPos(self,Pos):
    self.Port.motor.preset(int(Pos))
    self.PosAct=self.Port.device.get()[0]
  def GetPos(self):
    return self.Port.device.get()[0]
  def SetMode(self):
    self.Port.motor.mode(2)
def SaveVersion():
  f=open("ersver.txt", "w")
  ll=f.write("ERS_V3.0.12.20_Robot")
  f.close()
def SavePosAct():
  f=open("pos_act.json", "w")
  MotorPosActTmp = [0.0,0.0,0.0,0.0,0.0,0.0] 
  for i in range(6):
    if MRS[i]:
      MotorPosActTmp[i]=MPA[i]
    else:
      MotorPosActTmp[i]=-100000.0
  ll=f.write(json.dumps(MotorPosActTmp))
  f.close()
def Log():
  print("{T:1,S:"+ str(iStatus)+",R:"+json.dumps(MRS)+",CT:"+str(int(AllTicks/iLoops))+",BT:"+str(hub.battery.capacity_left())+",PA:"+ json.dumps(MPA)+",CC:" + str(hub.battery.info()["charge_current"])+",CS:" + str(hub.battery.info()["charger_state"])+",HT:" + str(hub.temperature())+",FV:\""+ sVerFs + "\"}")
def Clamp(Val,Max):
  if Val>Max:
    return Max
  elif Val<-Max:
    return -Max
  else:
    return Val
def SetStatus(status):
  global iStatus
  if iStatus!=status:
    iStatus=status
    hub.led(status)
SaveVersion()
hub.display.clear()
iStatus=0
SetStatus(7)
try:
  Fs=open("version.py")
  sVerFs=Fs.read()
  Fs.close()
  sVerFs=sVerFs.replace("\n", "").replace("\"", "")
  sVerFs=sVerFs[sVerFs.index("=")+2:]
  if sVerFs.find("-")>0:
    sVerFs=sVerFs[0:sVerFs.index("-")]
except:
  sVerFs="unknown"
CmdAct=0
CmdLast=0
iPosOld1=100000.0
iPosOld2=100000.0
SPR=[0.0,0.0,0.0,0.0,0.0,0.0]
APR=[1.2,1.2,-1.2,1.5,1.2,0.8]
APL=[2,2,2,3,2,5]
OldPoses=[0.0,0.0,0.0,0.0,0.0,0.0]
MRS=[False,False,False,False,False,False]
MPA=[0,0,0,0,0,0]
Mames=[0,6075,4725,0,0,0]
TOM=[25000,15000,15000,25000,30000,0]
WaitTick=0
MotorRefSum=0.0
MotorRefTicks=0
MotorOnBlock=0
AllTicks=0
iLoops=1
while True:
  mot=[]
  mot.append(Mot(hub.port.A))
  mot.append(Mot(hub.port.C))
  mot.append(Mot(hub.port.E))
  mot.append(Mot(hub.port.D))
  mot.append(Mot(hub.port.B))
  mot.append(Mot(hub.port.F))
  try:
    for i in range(6):
      iStatus=-1-i
      mot[i].SetMode()
      testp=mot[i].GetPos()
  except:
    Log()
    utime.sleep(1)
    continue
  try:
    f=open("pos_act.json")
    MotorPosActTmp=json.loads(f.read())
    for i in range(6):
      if MotorPosActTmp[i]>-100000.0:
        MRS[i]=True
        MPA[i]=MotorPosActTmp[i]
    f.close()
  except:
    MRS=[False,False,False,False,False,False]
  LastTick=0
  LastDataTick=0
  MotorsOn=False
  for i in range(6):
    mot[i].SetPos(MPA[i])
  LastDataTick=utime.ticks_ms()
  SetStatus(9)
  bMainLoop=True
  while bMainLoop:
    Tick=utime.ticks_ms()
    TickDiff=Tick-LastTick
    AllTicks=AllTicks+TickDiff
    LastTick=Tick
    LastDataTickDiff=Tick-LastDataTick
    if LastDataTickDiff>10000:
      if MotorsOn==True:
        SetStatus(9)
        MotorsOn=False
        CmdAct=0
        Log()
        SavePosAct()
        for i in range(6):
          mot[i].SetPwm(0)
      if hub.battery.charger_detect()==0:
        hub.power_off()
        hub.power_off(1)
    var1=hub_runtime.USB_VCP.readline()
    if var1 != None:
      try:
        val=json.loads(var1.decode("utf-8"))
        LastDataTick=Tick
        mot[0].SetPointIn=val["A1"]
        mot[1].SetPointIn=val["A2"]
        mot[2].SetPointIn=val["A3"]
        mot[3].SetPointIn=val["A4"]
        mot[4].SetPointIn=val["A5"]
        mot[5].SetPointIn=val["A6"]
        if val["On"]!=CmdLast:
          CmdLast=val["On"]
          CmdAct=CmdLast
      except:
        MotorsOn=False
    if CmdAct==1:
      if MotorsOn==False:
        SetStatus(6)
        MotorsOn=True
    elif CmdAct==0:
      for i in range(6):
        mot[i].SetPwm(0)
      SetStatus(9)
      for i in range(6):
        if MRS[i]==False:
          try:
            mot[i].SetPos(mot[i].SetPointIn)
          except:
            MPA[i]=0
            MotorsOn=False
            bMainLoop=False
            break
      if MotorsOn==True:
        MotorsOn=False
        SavePosAct()
    elif CmdAct>1:
      if CmdAct==10:
        MotorsOn=False
        SetStatus(9)
        try:
          os.remove("pos_act.json")
        except:
          ll=0
        MRS=[False,False,False,False,False,False]
        CmdAct=0
      elif CmdAct>20 and CmdAct<27:
        MotorsOn=False
        SetStatus(3)
        iAxis=CmdAct-21
        m=mot[iAxis]
        MRS[iAxis]=False
        m.SetPos(0)
        if iAxis==1:
          m.MovePosWait(-550,25)
        if iAxis==2:
          m.MovePosWait(550,25)
        utime.sleep(0.1)
        m.SetPwm(0)
        m.SetPos(0)
        m.Zero()
        m.PosAct=0
        utime.sleep(0.1)
        SPR[iAxis]=0.0
        MotorRefSum=0.0
        MotorRefTicks=0
        WaitTick=utime.ticks_ms()
        CmdAct=31+iAxis
      elif (CmdAct>30 and CmdAct<37) or (CmdAct>40 and CmdAct<47):
        MotorsOn=True
        iAxis=CmdAct-31
        addPointRef=0.0
        if CmdAct>40:
          iAxis=CmdAct-41
          addPointRef=APR[iAxis]*(-1)
        else:
          addPointRef=APR[iAxis]
        m=mot[iAxis]
        for i in range(6):
          mot[i].SetPointIn=mot[i].PosAct
        SPR[iAxis]=SPR[iAxis]+addPointRef
        m.SetPointIn=SPR[iAxis]
        if (m.Max==100.0) and (abs(m.PosAct)>150):
          MotorRefSum+=abs(m.PwmAct)
          MotorRefTicks+=1
          MotorOnBlock=0
        if (m.Max==100.0) and (abs(m.PosAct)>400):
          m.Max=abs(MotorRefSum/MotorRefTicks)+APL[iAxis]
        if (m.Max<100) and (m.Max<=abs(m.PwmAct)):
          MotorOnBlock+=1
        else:
          MotorOnBlock=0
        if (MotorOnBlock>70):
          m.SetPwm(0)
          MotorOnBlock=0
          if CmdAct<40:
            iPosOld1=m.PosAct
          else:
            iPosOld2=m.PosAct
          if (CmdAct<40) and ((iAxis==0) or (iAxis==4)):
            m.MovePosWait(0,25)
            CmdAct=41+iAxis
          else:
            CmdAct=51+iAxis
          m.Zero()
          m.SetPointIn=m.GetPos()
          SPR[iAxis]=m.SetPointIn
          MotorRefSum=0.0
          MotorRefTicks=0
        if (WaitTick+TOM[iAxis])<utime.ticks_ms():
          if abs(m.PosAct)>300:
            iStatus=101
          else:
            iStatus=102
          utime.sleep(0.100)
          Log()
          utime.sleep(0.100)
          CmdAct=0
      elif CmdAct>50 and CmdAct<57:
        MotorsOn==False
        iAxis=CmdAct-51
        m=mot[iAxis]
        m.SetPwm(0)
        if iAxis==0:
          m.MovePosWait(int((iPosOld1+iPosOld2)/2)+50,25)
        if iAxis==1:
          m.MovePosWait(int(iPosOld1)-420,25)
        if iAxis==2:
          m.MovePosWait(int(iPosOld1)+260,25)
        if iAxis==3:
          m.MovePosWait(int(iPosOld1)-1730,25)
        if iAxis==4:
          m.MovePosWait(int((iPosOld1+iPosOld2)/2)-60,25)
        m.Zero()
        m.SetPos(Mames[iAxis])
        MRS[iAxis]=True
        MPA[iAxis]=m.GetPos()
        SavePosAct()
        SetStatus(9)
        CmdAct=0
      elif (CmdAct>10) and (CmdAct<17):
        MotorsOn==False
        SetStatus(9)
        iAx=CmdAct-11
        MRS[iAx]=False
        mot[iAx].SetPos(Mames[iAx])
        utime.sleep(0.2)
        MPA[iAx]=mot[iAx].GetPos()
        MRS[iAx]=True
        SavePosAct()
        CmdAct=0
      elif (CmdAct>60) and (CmdAct<67):
        MotorsOn==False
        SetStatus(9)
        iAx=CmdAct-61
        MRS[iAx]=False
        if iAx==3:
          MRS[iAx+1]=False
          MRS[iAx+2]=False
        if iAx==4:
          MRS[iAx+1]=False
        SavePosAct()
        CmdAct=0
      elif CmdAct==20:
        MotorsOn==False
        SetStatus(9)
        for i in range(6):
          MRS[i]=False
          mot[i].SetPos(mot[i].SetPointIn)
          MPA[i]=mot[i].GetPos()
          MRS[i]=True
        SavePosAct()
        CmdAct=0
      elif CmdAct==2:
        MotorsOn==False
        SetStatus(9)
        SavePosAct()
        iStatus=0
        MRS=[False,False,False,False,False,False]
        Log()
        hub.power_off()
        hub.power_off(1)
        break
    for i in range(6):
      try:
        MPA[i]=mot[i].Exe(MotorsOn,TickDiff)
      except:
        MRS[i]=False
        MPA[i]=0
        MotorsOn=False
        bMainLoop=False
        break
    if AllTicks>1000:
      if MotorsOn:
        for i in range(6):
          if (abs(mot[i].CurrentError)>3600):
            MotorsOn=False
            bMainLoop=False
            iStatus=111+i
      Log()
      AllTicks=0
      iLoops=0
    iLoops=iLoops+1
  for i in range(6):
    mot[i].SetPwm(0)
  utime.sleep(1)
  SetStatus(9)
  CmdAct=0;
  SavePosAct()
