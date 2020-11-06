// Created for CSCI 5611 by Liam Tyler

// WASD keys move the camera relative to its current orientation
// Arrow keys rotate the camera's orientation
// Holding shift boosts the move speed

/*

  Code taken from the example code in course's Canvas site. We don't take any authorship
  in this code.
  
  - Jeffrey Jia

*/

class Camera
{
  Camera()
  {
    position      = new PVector(512, 384, 870); // initial position
    theta         = 0; // rotation around Y axis. Starts with forward direction as ( 0, 0, -1 )
    phi           = 0; // rotation around X axis. Starts with up direction as ( 0, 1, 0 )
    moveSpeed     = 12.5;
    turnSpeed     = .01; // radians/sec
    boostSpeed    = 5;  // extra speed boost for when you press shift
    
    // dont need to change these
    shiftPressed = false;
    negativeMovement = new PVector( 0, 0, 0 );
    positiveMovement = new PVector( 0, 0, 0 );
    negativeTurn     = new PVector( 0, 0 ); // .x for theta, .y for phi
    positiveTurn     = new PVector( 0, 0 );
    fovy             = PI / 4;
    aspectRatio      = width / (float) height;
    nearPlane        = 0.1;
    farPlane         = 10000;
  }
  
  void Update(float dt)
  {
    theta += turnSpeed * ( negativeTurn.x + positiveTurn.x)*dt;
    
    // cap the rotation about the X axis to be less than 90 degrees to avoid gimble lock
    float maxAngleInRadians = 85 * PI / 180;
    phi = min( maxAngleInRadians, max( -maxAngleInRadians, phi + turnSpeed * ( negativeTurn.y + positiveTurn.y ) * dt ) );
    
    // re-orienting the angles to match the wikipedia formulas: https://en.wikipedia.org/wiki/Spherical_coordinate_system
    // except that their theta and phi are named opposite
    float t = theta + PI / 2;
    float p = phi + PI / 2;
    PVector forwardDir = new PVector( sin( p ) * cos( t ),   cos( p ),   -sin( p ) * sin ( t ) );
    PVector upDir      = new PVector( sin( phi ) * cos( t ), cos( phi ), -sin( t ) * sin( phi ) );
    PVector rightDir   = new PVector( cos( theta ), 0, -sin( theta ) );
    PVector velocity   = new PVector( negativeMovement.x + positiveMovement.x, negativeMovement.y + positiveMovement.y, negativeMovement.z + positiveMovement.z );
    position.add( PVector.mult( forwardDir, moveSpeed * velocity.z * dt ) );
    position.add( PVector.mult( upDir,      moveSpeed * velocity.y * dt ) );
    position.add( PVector.mult( rightDir,   moveSpeed * velocity.x * dt ) );
    
    aspectRatio = width / (float) height;
    perspective( fovy, aspectRatio, nearPlane, farPlane );
    camera( position.x, position.y, position.z,
            position.x + forwardDir.x, position.y + forwardDir.y, position.z + forwardDir.z,
            upDir.x, upDir.y, upDir.z );
  }
  
  // only need to change if you want diffrent keys for the controls
  void HandleKeyPressed()
  {
    if ( key == 'q' || key == 'Q' ) positiveMovement.z = 1;
    if ( key == 'e' || key == 'E' ) negativeMovement.z = -1;
    if ( key == 'h' || key == 'H' ) positiveMovement.x = 1;
    if ( key == 'f' || key == 'F' ) negativeMovement.x = -1;
    if ( key == 't' || key == 'T' ) positiveMovement.y = 1;
    if ( key == 'g' || key == 'G' ) negativeMovement.y = -1;
    
    if ( key == 'z' || key == 'Z' )
    {
      Camera defaults = new Camera();
      position = defaults.position;
      theta = defaults.theta;
      phi = defaults.phi;
    }
    
    if ( key == 'a' || key == 'A' )  negativeTurn.x = 1;
    if ( key == 'd' || key == 'D' ) positiveTurn.x = -1;
    if ( key == 'w' || key == 'W' )    positiveTurn.y = 1;
    if ( key == 's' || key == 'S'  )  negativeTurn.y = -1;
    
    if ( keyCode == SHIFT ) shiftPressed = true; 
    if (shiftPressed){
      positiveMovement.mult(boostSpeed);
      negativeMovement.mult(boostSpeed);
    }
    
  }
  
  // only need to change if you want difrent keys for the controls
  void HandleKeyReleased()
  {
    if ( key == 'q' || key == 'Q' ) positiveMovement.z = 0;
    if ( key == 'e' || key == 'E' ) negativeMovement.z = 0;
    if ( key == 'h' || key == 'H' ) positiveMovement.x = 0;
    if ( key == 'f' || key == 'F' ) negativeMovement.x = 0;
    if ( key == 't' || key == 'T' ) positiveMovement.y = 0;
    if ( key == 'g' || key == 'G' ) negativeMovement.y = 0;
    
    if ( key == 'a' || key == 'A' ) negativeTurn.x = 0;
    if ( key == 'd' || key == 'D' ) positiveTurn.x = 0;
    if ( key == 'w' || key == 'W' ) positiveTurn.y = 0;
    if ( key == 's' || key == 'S' ) negativeTurn.y = 0;
    
    if ( keyCode == SHIFT ){
      shiftPressed = false;
      positiveMovement.mult(1.0/boostSpeed);
      negativeMovement.mult(1.0/boostSpeed);
    }
  }
  
  // only necessary to change if you want different start position, orientation, or speeds
  PVector position;
  float theta;
  float phi;
  float moveSpeed;
  float turnSpeed;
  float boostSpeed;
  
  // probably don't need / want to change any of the below variables
  float fovy;
  float aspectRatio;
  float nearPlane;
  float farPlane;  
  PVector negativeMovement;
  PVector positiveMovement;
  PVector negativeTurn;
  PVector positiveTurn;
  boolean shiftPressed;
};
