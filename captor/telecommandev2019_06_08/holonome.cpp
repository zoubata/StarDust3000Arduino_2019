/*
holonome(cmd123)
setVxy(x,y,theta)
setDxy(x,y,theta)
setVdo(d,theta)=>setVxy(x,y,0)
setDdo(d,theta)
getXYO()

void Computexyt(signed int *dx,signed int *dy,signed int *dt)
{
  for(int i=0;i<NbW;i++)
  {
    do2[i]=cmd[i]->getEncoder().getValue()*2*PI*Diam[i]/Inc[NbW];
  }
  ddo[]=d2o[]-do[];
  do[]=d2o[];
  X
  Y
  *dt=0;
    for(int i=0;i<NbW;i++)
      *dt+=ddo[]*360/(Di[]*PI*2)/3;
      
  *dx=0;
    for(int i=0;i<NbW;i++)
      *dx+=ddo[]*cos(????)
      
      dy....
}
#define NB_W_MAX 6
signed int Yi[NB_W_MAX]; //!< coefficent for Y axes for wheel i
signed int Ti[NB_W_MAX]; //!< coefficent for theta rotation for wheel i
signed int Di[NB_W_MAX]; //!< distance of wheel to  the ceter of robot.
signed int Xi[NB_W_MAX]; //!< coefficent for X axes for wheel i
signed int v[NB_W_MAX]; //!< speed of wheel i
signed int d[NB_W_MAX]; //!< distance of wheel i


  /**
  axes are define as this :
  
  Y
  ^
  |---.
  |    \
  |     )90�
  |      \ 
  +---------> X
  * /

 NbW=0;
  */
  /** a a wheel at corrodinate x,y,theta.
  
  * /
void addMotor(*cmdi,singed int xmm, signed int ymm, signed int thetaDegree, unsigned int diam,unsigned ind nbIncPerRound)
{
  Wx[NbW]=xmm;
  Wy[NbW]=ymm;
  Wtheta[NbW]=thetaDegree;
  Diam[NbW]=diam;
  cmd[NbW]=cmdi;  
  Inc[NbW]=nbIncPerRound
  NbW++;
}
/** precompute projection
D=(x*x+y*y)^0.5
  v1=v2=v3 =theta*2*PI()*D/360;//theta degre    
           =theta*Ti
  v1=v2=v3 +=y/cos(teta) si cos(tetha)!=0;sinon 0    
           +=y*Yi         
  v1=v2=v3 +=x*cos(teta-90)                           *Xi
           +=x*Xi
  Xi,Yi,Ti is float at fix point : /1024
* /
void precompute()
{
  for(int i=0;i<NbW;i++)
  {
    Di[i]=sqrt(Wx[i]*Wx[i]+Wy[i]*Wy[i]);
    Ti[i]=1024*Wtheta[i]*2*PI()*Di[i]/360;
      if(Wtheta[i]!=0)
        Yi[i]=1024/cos(Wtheta[i]);
      else
        Yi=0;
    Xi[i]=1024*cos(Wtheta[i]-90) ;
  }
}

/** set the speed of robot : teta in nb tick for 360�

compute vi as int 
* /
void setVxyo(signed int  x,signed int y,signed int teta)
{
  for(int i=0;i<NbW;i++)
  {
    v[i]=theta*Ti[i];
    v[i]+=y*Yi[i];
    v[i]+=x*Xi[i];
    v[i]=v[i]>>10;// "/1024"=">>10"
  }
  for(int i=0;i<NbW;i++)
    cmd[i]->setPoint(v[i]);
  
}
/** set the distance of robot : teta in nb tick for 360�
compute di as int 
* /
void setDxyo(signed int x,signed int y,signed int teta)
{
  for(int i=0;i<NbW;i++)
  {
    d[i]+=theta*Ti[i];
    d[i]+=y*Yi[i];
    d[i]+=x*Xi[i];
    d[i]=d[i]>>10;// "/1024"=">>10"
  }
}

void loop()
{
  for(int i=0;i<NbW;i++)
    cmd[i].loop();
}

asser v123

asser theta,d

inertie=force=Mdv
PWM=force
*/
