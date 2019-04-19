// Shawn Daniel
// Analyzed 4/25/2016

class Platform {
  private PVector translation, rotation, initialHeight, mball;  
  private PVector[] baseJoint, platformJoint, q, l, A, ballPos; 
  private float[] alpha; 
  private float baseRadius, platformRadius, hornLength, legLength;  

  private final float baseAngles[] = {    
   314.9, 345.1, 74.9, 105.1, 194.9, 225.1 };   // Look at the base angles of the sim.O

  private final float platformAngles[]  = {  
   322.9, 337.1, 82.9, 97.1, 202.9, 217.1};

   private final float beta[] = {
   -8*PI/3, PI/3, 0, -PI, -4*PI/3, -7*PI/3}; 
   
  // REAL MEASUREMENTS: 
  private final float SCALE_INITIAL_HEIGHT = 120; 
  private final float SCALE_BASE_RADIUS = 65.43; 
  private final float SCALE_PLATFORM_RADIUS = 76.35;
  private final float SCALE_HORN_LENGTH = 36;  
  private final float SCALE_LEG_LENGTH = 125+3; 

  public Platform(float s) {  
  // s=1.    
  
  // Build the robot.
    translation = new PVector();
    mball  = new PVector();
    initialHeight = new PVector(0, 0, s*SCALE_INITIAL_HEIGHT); 

    rotation = new PVector(); 
    baseJoint = new PVector[6];
    platformJoint = new PVector[6];

    alpha = new float[6]; 
    q = new PVector[6]; 
    l = new PVector[6];
    A = new PVector[6]; 
    ballPos= new PVector[1];
    
    baseRadius = s*SCALE_BASE_RADIUS;
    platformRadius = s*SCALE_PLATFORM_RADIUS;
    hornLength = s*SCALE_HORN_LENGTH;
    legLength = s*SCALE_LEG_LENGTH;

    for (int i=0; i<6; i++) {  
      float mx = baseRadius*cos(radians(baseAngles[i])); 
      float my = baseRadius*sin(radians(baseAngles[i]));
      baseJoint[i] = new PVector(mx, my, 0); 
    } //draw

    for (int i=0; i<6; i++) { 
     float mx = platformRadius*cos(radians(platformAngles[i])); 
     float my = platformRadius*sin(radians(platformAngles[i]));
	 
      platformJoint[i] = new PVector(mx, my, 0);  
      q[i] = new PVector(0, 0, 0); 
      l[i] = new PVector(0, 0, 0);
      A[i] = new PVector(0, 0, 0); 
      ballPos[0]= new PVector(0,0,0); 
    }
    calcQ(); 
  }

// Below is mega function using all tghe inverse kinematic equations to spit out control for desired T/R.  
  public void applyTranslationAndRotation(PVector t, PVector r, PVector y) {  
    rotation.set(r); 
    translation.set(t);
    mball.set(y);
    
    calcQ(); 
    calcAlpha(); 
  }

  private void calcQ() {        
  // Output:  Neccesary: l[i] q[i] as functions of desired "translation" and "rotation" predeclared PVector values.
  // Rotation charactereized by 3-2-1 rotation sequence.
 
    for (int i=0; i<6; i++) {   // Runs for 6-iterations. 
      // rotation, of x,y,z of a PVector elements.
	  // Note: Calculate "q" for each platform joint. 
	  // q[i]= (x,y,z),  q is 6, Pvectors.
	  
	  // q=R*p; Note initially 0-vector translation and rotation angles so R is identity.
      q[i].x = cos(rotation.z)*cos(rotation.y)*platformJoint[i].x +                 								
        (-sin(rotation.z)*cos(rotation.x)+cos(rotation.z)*sin(rotation.y)*sin(rotation.x))*platformJoint[i].y +		
        (sin(rotation.z)*sin(rotation.x)+cos(rotation.z)*sin(rotation.y)*cos(rotation.x))*platformJoint[i].z;		

      q[i].y = sin(rotation.z)*cos(rotation.y)*platformJoint[i].x +
        (cos(rotation.z)*cos(rotation.x)+sin(rotation.z)*sin(rotation.y)*sin(rotation.x))*platformJoint[i].y +		
        (-cos(rotation.z)*sin(rotation.x)+sin(rotation.z)*sin(rotation.y)*cos(rotation.x))*platformJoint[i].z;		

      q[i].z = -sin(rotation.y)*platformJoint[i].x +						
        cos(rotation.y)*sin(rotation.x)*platformJoint[i].y +				
        cos(rotation.y)*cos(rotation.x)*platformJoint[i].z;					
      
      ballPos[0].x =  mball.x;     
     
      ballPos[0].y = mball.y; 
      
      ballPos[0].z =  0;  

      // translation added onto "q"
	  //  q= Translation+initialHeight+R*p;
      q[i].add(PVector.add(translation, initialHeight));  
      l[i] = PVector.sub(q[i], baseJoint[i]); // Awesome! Yes.
	  // NOTE, the skeleton for the platform rigid body and Base is already made by the constants: baseAngles, platformAngles.
	  // The inverse kinematics must be drawn. SCD YES! 9/19/2018
    }
  }

  private void calcAlpha() {  // calculate "alpha" based on current translatiom  and rotation angles of platform.
                              // We need required alpha[i] for fitting the simulation.
  
  // Reference my notes.  This is for the special servo horn drawings. Over complicated!
    for (int i=0; i<6; i++) {    
      float L = l[i].magSq()-(legLength*legLength)+(hornLength*hornLength); // l^2-(s^2+a^2). Yes.
      float M = 2*hornLength*(q[i].z-baseJoint[i].z); // 2*a*(q.z-b.z)
      float N = 2*hornLength*(cos(beta[i])*(q[i].x-baseJoint[i].x) + sin(beta[i])*(q[i].y-baseJoint[i].y)); // 2*a*(cos(Beta)*(p.x-b.x)+sin(Beta)*(p.y-b.y))
      alpha[i] = asin(L/sqrt(M*M+N*N)) - atan2(N, M); // 

	  // The servo horn vector calculation.
      A[i].set(hornLength*cos(alpha[i])*cos(beta[i]) + baseJoint[i].x,     
      hornLength*cos(alpha[i])*sin(beta[i]) + baseJoint[i].y, 
      hornLength*sin(alpha[i]) + baseJoint[i].z);// A[i].set = (. ,. ,.); // Solved in vector form! This is correct.

      float xqxb = (q[i].x-baseJoint[i].x); // q.x-b.x
      float yqyb = (q[i].y-baseJoint[i].y); // q.y-b.y
      float h0 = sqrt((legLength*legLength)+(hornLength*hornLength)-(xqxb*xqxb)-(yqyb*yqyb)) - q[i].z; // sqrt( s^2+a^2-(q.x-b.x)^2-(q.y-b.y)^2)-q.z

      float L0 = 2*hornLength*hornLength; // L
      float M0 = 2*hornLength*(h0+q[i].z);
      float a0 = asin(L0/sqrt(M0*M0+N*N)) - atan2(N, M0); // Home position. wtf to the M and N values in the equation. Nope it's correct.

      //println(i+":"+alpha[i]+"  h0:"+h0+"  a0:"+a0);
    }
  }
  
  public float[] getAlpha(){  // Return the alpha. "It's a private variable!!!"
    return alpha; 
  }
  
  //----------------------//
  
  
// DRAW THE SYSTEM.. //
  public void draw() {   // Here we draw the object! Simulator.pde is use for controlling the object.
    // draw Base. Yes.
    noStroke(); // no outline, onlt the fill of shapes will be drawn
    fill(128);
    ellipse(0, 0, 2*baseRadius, 2*baseRadius); /* Note: drawing the Base only serves for painting pretty picture, it could be any shap, with no structural significance.
           Note that the calculation of actual values such as Servo values and sending these numerical values to program
           with physical meaning like arduino or MATLAB will allow simulation controlled motors of physical model. With careful design
           Simulation could be a ditto picture of concurrent controlled physical system. */ 
		   
		   
	// Below shift datum drawing of base image to print values of each alpha angle at corresponding base joint location.
    for (int i=0; i<6; i++) {
      pushMatrix();
						// Offset by the same translation.
      translate(baseJoint[i].x, baseJoint[i].y, baseJoint[i].z); //Shift drawing datum 3D point of each Base joint
      noStroke();
      fill(0); // Fill all.   
      ellipse(0, 0, 7, 7); // So we get black spheres representing base joints.
      text(String.format("%.2f", degrees(alpha[i])), 5,5,5);  // Here is listed text x,y,z positions wrt respective base joint.
      popMatrix();

      stroke(245);  // White outline.
      line(baseJoint[i].x, baseJoint[i].y, baseJoint[i].z, A[i].x, A[i].y, A[i].z); // Draw servo horn actuator. Yes.
     // Continue here.
     
      PVector rod = q[i];
      
      stroke(100);
      strokeWeight(3); // Thick servo arm.
      line(A[i].x, A[i].y, A[i].z, rod.x, rod.y, rod.z);
    }

    // draw phone joints and rods. On the platform.
    for (int i=0; i<6; i++) {
      pushMatrix(); // Set the translation from the base.
      translate(q[i].x, q[i].y, q[i].z); // So we in "q" coordinate for each reference frame of each joint.
      noStroke(); // No outline.
      fill(0);
      ellipse(0, 0, 5, 5); // The small black spheres representing connections.
      popMatrix(); //  Exit translation.

	  // Make line l(i) Image with
      stroke(100);
      strokeWeight(1); // thin line.
      line(baseJoint[i].x, baseJoint[i].y, baseJoint[i].z, q[i].x, q[i].y, q[i].z); // done for each of 6 joints separately.
    }

    // sanity check. Draw platform.sss.
    pushMatrix();
    translate(initialHeight.x, initialHeight.y, initialHeight.z); // Shifted datum to initial height. // I.C. or zero out the platform.
    translate(translation.x, translation.y, translation.z); 

    rotateZ(rotation.z); 
    rotateY(rotation.y);
    rotateX(rotation.x);
    
    pushMatrix();
        translate(0,0,9); 
        fill(0);
        ellipse(0,0,20,20); 
        translate(0,0,1); 
        translate(mball.x, mball.y,0);
        noStroke(); 
        fill(254);
        sphere(10);
    popMatrix();
    
    stroke(245);
    fill(10);
    // noFill();  
    ellipse(0, 0, 2*platformRadius, 2*platformRadius);  // Platform drawing
    popMatrix(); // Note: we painted the ellipse in a translated and rotated datum reference frame.
  }
}