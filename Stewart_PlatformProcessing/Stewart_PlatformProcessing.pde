  // Shawn Daniel  
  // Edited 9/19/2018  
  
  import peasy.*; 
  import controlP5.*;  
  import oscP5.*;
  import netP5.*; 
  
  
  
  float MAX_TRANSLATION = 1;  
  float MAX_ROTATION = 1;
  float MAX_BALL_TRANSLATION = 1;  
  float ddang1x,ddang1y, ddx, ddy; 
  float dt1= .07; 
  boolean llave, llave2, llave3,llave4,llave5,llave6,llave7,llave8;

  
   // DYNAMICS PARAMETERS

   float g = 9.81;
   float  ang1x = 0*PI/180;  
   float dang1x = 0; 
   float  ang1y = 0*PI/180;  
   float dang1y = 0; 
   float theta1=0;
   float theta2=0;
   float angMax= 20;
   float x=0;    
   float dx=0;
   float y=0;
   float dy=0;
   float cntrlx=0;
   float cntrly=0;
   
   // Initiate SMC and Continuous SMC.
    float Asmc = 1;
    float Bsmc = 1;
    float manifold = 0.0;
    float smcCntrl = 0.0; 
    float epsilon = 1.0;

   // Controller Parameters
     float k1= .5050; 
     float k2= 2.505;  
     float k3= -10.00;  
     float k4= -5.0; 
     
     // Gains for FB Linearization.
     float k1t= 1.0;  
     float k2t= 1.0;  
     float k3t= 1.0; 
     float k4t= 1.0; 
     
     // Control Regime Saturation.
     float satur = 0.05 ;  
     float satur2= 2.0  ;
     float satur3= 6.5;
   
   // Friction
     float frict= 3;
   
   // Lyapunov Based Control
     float ke=1;
   
   
  ControlP5 cp5;
  PeasyCam camera; 
  Platform mPlatform;  
  OscP5 oscP5;
  NetAddress mOscOut; 
  
  float posX=0, posY=0, posZ=0, pitch=0, rotY=0, rotZ=0, refx=0,refy=0, perturb=0; 
  float ballX=0, ballY=0, ballZ=0; 
  boolean ctlPressed = false;
  
  
  
  void setup() {
    size(768, 768, P3D); // Form 3D window.
    smooth();
    frameRate(60); // update frame 60 times a second.
    textSize(20); 
  
    mOscOut = new NetAddress("192.168.0.24", 8888);
  
    camera = new PeasyCam(this, 650); // Initial zoom onto the target object.
    
    camera.setRotations(-1.0, 0.0, 0.0); // set initial camera rotation view of object.
    camera.lookAt(8.0, -50.0, 80.0); // our set initial view.
  
    mPlatform = new Platform(1); 
    mPlatform.applyTranslationAndRotation(new PVector(), new PVector(), new PVector()); 
    cp5 = new ControlP5(this); // Create control object.
    
    // Note: Unless correctly names, control sliders here in the GUI won't edit object desired parameters.
    cp5.addSlider("posX").setPosition(20, 20).setSize(180, 40).setRange(-1, 1); 
    cp5.addSlider("posY").setPosition(20, 70).setSize(180, 40).setRange(-1, 1);
    cp5.addSlider("posZ").setPosition(20, 120).setSize(180, 40).setRange(-1, 1);
  
    cp5.addSlider("ballX").setPosition(20, 170).setSize(180, 40).setRange(-1, 1); 
    cp5.addSlider("ballY").setPosition(20, 215).setSize(180, 40).setRange(-1, 1);
    cp5.addSlider("ballZ").setPosition(20, 260).setSize(180, 40).setRange(-1, 1);
    cp5.addSlider("perturb").setPosition(20, 305).setSize(180, 40).setRange(-1, 1);
    
    cp5.addSlider("pitch").setPosition(width-200, 20).setSize(180, 40).setRange(-1, 1); // width: 1024
    cp5.addSlider("rotY").setPosition(width-200, 70).setSize(180, 40).setRange(-1, 1);
    cp5.addSlider("rotZ").setPosition(width-200, 120).setSize(180, 40).setRange(-1, 1);
    cp5.addSlider("refx").setPosition(width-200, 170).setSize(180, 40).setRange(-50, 50); 
    cp5.addSlider("refy").setPosition(width-200, 220).setSize(180, 40).setRange(-50, 50);
  
    cp5.setAutoDraw(false); 
    camera.setActive(true); 
    
    // Dynamics Simulation.
    llave= false; // For triggering the dynamic laws on the system.
    llave2= false; // For triggering control system.
    llave3= false; // For triggering Full State Feedback.
    llave4= false; // For triggering perturbations.
    llave5= false; // For tracking reference trajectory.
    llave6= false; // For triggering Hybrid passivity Lyapunov and LQR local asymptotic stability controller.
    llave7= false; // For triggering Feedback Linearzation. 
    llave8= false; //  Continuous SMC.
    
  }
  
  
 
  void draw() {  //Loop Mechanism.
    background(200); 
    mPlatform.applyTranslationAndRotation(PVector.mult(new PVector(posX, posY, posZ), MAX_TRANSLATION), 
      PVector.mult(new PVector(pitch, rotY, rotZ), MAX_ROTATION),
      PVector.mult(new PVector(ballY, ballX, 0), MAX_BALL_TRANSLATION)); 
 
    mPlatform.draw(); 
  
    hint(DISABLE_DEPTH_TEST); 
    
    camera.beginHUD(); 

    cp5.draw(); 
    
    camera.endHUD();
    
    hint(ENABLE_DEPTH_TEST); // Play with this.
    
  // BELOW TURNS SIMULATION ENVIRONMENT ON IF KEYS AND SWITCHED.
  if(llave){  // Apply dynamic laws on the system.
     
      rotY= -ang1y;  // Activate Dynamics
      pitch= ang1x;
      ballX=x;
      ballY=y;
    
      //  Keep the ball on the platform.

     if(x>50){  
       x=50;
       
       ddx=0; 
       dx=0;  
     }
     if(x<-50){
       x= -50;
       ddx=0;
       dx=0;
     }
     if(y>50){  
       y=50;
       ddy=0;  
       dy=0;  
     }
     if(y<-50){
       y= -50;
       ddy=0;
       dy=0;
     }
    
     // Put saturation on 'x' here and insure it doesn't interferee with the simulation.
      text(String.format("%.2f", x), 75,-10,-10); 
      text(String.format("%.2f", y), 75,20,-10); 
   
 // NOTE: DECOUPLED DYNAMICS FOR PITCH AND ROLL BALL POSITION TRACKING
    ang1x += dang1x*dt1; 
     // angle saturater just after angle update.
     if (ang1x>angMax*PI/180) {
       ang1x=angMax*PI/180;
       dang1x=0;
       ddang1x=0;
       cntrlx=0;
     }
     if (ang1x<-angMax*PI/180) {
       ang1x=-angMax*PI/180;
       dang1x=0;
       ddang1x=0;
       cntrlx=0;
     }
    dang1x = dang1x + ddang1x*dt1;  
    ddang1x = cntrlx;     
    x = x + dx*dt1;   
    dx += ddx*dt1;
    ddx= ( 0*x*dang1x*dang1x-g*sin(ang1x))  - frict*dang1x; /* eliminate corriolis/centripetal acceleration for aid in simulaiton. Note: inertia of the rolling ball 
    will decrease centripetal acclleraiton abit. Look at otherEOM information of the ball on platform system. */
    
    
    ang1y += dang1y*dt1; 
     // angle saturate just after  update.
     if (ang1y>angMax*PI/180) {
       ang1y=angMax*PI/180;
       dang1y=0;
       ddang1y=0; 
       cntrly=0;
     }
     if (ang1y<-angMax*PI/180) {
       ang1y=-angMax*PI/180;
       dang1y=0;
       ddang1y=0;
       cntrly=0;
     }
    dang1y = dang1y + ddang1y*dt1;  
    ddang1y = cntrly;     
    y = y + dy*dt1;   
    dy += ddy*dt1;
    ddy= ( 0*y*dang1y*dang1y-g*sin(ang1y))  - frict*dang1y; 
    }
  
     // Below is control activation key.
      if (llave2 == false){  
        cntrlx=0; 
        cntrly=0;
      }
    
    if(llave3 & llave2){    // Turn on control Linear Full state Feedback.
      
          if(abs(ang1x)!=angMax) {   
             cntrlx= 1*(k1*(x-refx)+k2*dx+k3*ang1x+k4*dang1x);  // Tracking regulation added. 
          }
          if(abs(ang1y)!=angMax) {   
             cntrly= 1*(k1*(y-refy)+k2*dy+k3*ang1y+k4*dang1y);
          }
    }
   
    // View perturbation. Not complete.
      if(llave4 & llave2){       
       ddx=ddx+20*perturb;  // add a scalled steady state disturbance. 
      }
      
      // Initiate TRACK REFERENCE TRAJECTORY here.
      if(llave5 & llave2) {
        refx = 50*cos(theta1);  
        refy = 50*sin(theta2);
        theta1 += PI/100.0;
        theta2 += PI/100.0;     
      }
   }   

  void controlEvent(ControlEvent theEvent) {
    camera.setActive(false);  
  }
  
  void mouseReleased() {
    camera.setActive(true); 
  }
  
  long lastTime = 0; 
   
  void mouseDragged ()  
  {   
  
    if (ctlPressed) {   
      posX = map(mouseX, 0, width, -1, 1);                                        
      posY = map(mouseY, 0, height, -1, 1);
    }
  }
  
  // Analyse here.
  void keyPressed() {   
    if (key == ' ') {
      camera.setRotations(-1.0, 0.0, 0.0);   
      camera.lookAt(8.0, -50.0, 80.0); 
      camera.setDistance(650);      
    } else if (keyCode == CONTROL) { 
      camera.setActive(false);
      ctlPressed = true;
      llave=  false;  
      llave2= false;
      llave3= false;
      llave4= false;
      llave5= false;
      llave6= false;
      llave7= false;
      llave8= false;
    }
    
    // BELOW WE SWITCH CERTAIN SIMULATION MODES ON AND OFF.
    else if (key == 'g'){  
      llave=true;        
      } 
      else if (key == 'h'){
       llave2=true; 
      }
       else if (key == 'j'){
       llave3=true; 
      }
        else if (key == 'k'){
       llave4=true; 
      }
        else if (key == 'l'){
       llave5=true; 
      }
        else if (key == 'm'){
       llave6=true; 
      }
        else if (key == 'n'){
       llave7=true; 
      }
        else if (key == 'b'){
       llave8=true; 
      }
      else if (key == 'G'){
       llave=false; 
      }
      else if (key == 'H'){
       llave2=false; 
      }
      else if (key == 'J'){
       llave3=false; 
      }
      else if (key == 'K'){
       llave4=false; 
      }
      else if (key == 'L'){
       llave5=false; 
      }
      else if (key == 'M'){
       llave6=false; 
      }
      else if (key == 'N'){
       llave7=false; 
      }
      else if (key == 'B'){
       llave8=false; 
      }
  }
  
  void keyReleased() {   
    if (keyCode == CONTROL) {
      camera.setActive(true);
      ctlPressed = false;
    }
  }
  
  
  
  
  
  
  
  
  
