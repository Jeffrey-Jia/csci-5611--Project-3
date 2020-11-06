//CSCI 5611 - Project 3
//Jeffrey Jjia <jiaxx215>
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests


Camera camera;

//Change the below parameters to change the scenario/roadmap size
int numObstacles = 10;
int numNodes  = 999;
  
//A list of circle obstacles
static int maxNumObstacles = 15;
Vec3 circlePos[] = new Vec3[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii
int circleTexture[] = new int[maxNumObstacles];
boolean generate_obstacle = false;
PShape globe;

Vec3 startPos = new Vec3(100,500,0);
Vec3 goalPos = new Vec3(500,200,0);
Vec3 target;

static int maxNumNodes = 1000;
Vec3[] nodePos = new Vec3[maxNumNodes];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec3[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec3 randPos = new Vec3(random(width),random(height),0);
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec3(random(width),random(height),0);
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec3(random(100,900),random(100,650),0);
    //circleRad[i] = (50+20*pow(random(1),4));
    circleRad[i] = (50+10*random(2));
    circleTexture[i] = (int)random(0,4);
  }
  circleRad[0] = 60; //Make the first obstacle big
}

//The agent we are controlling
float old_rotate = 0.0;
float rotate = 0.0;
float agentRad = 10;
Vec3 agentPos = startPos;
Vec3 agentVel = new Vec3(200,40,0);
float goalSpeed = 40;

int curPathIdx = 0;

ArrayList<Integer> curPath;

PImage space_background;
PImage rocketship;
PImage asteroid;
PImage earth;
PImage satellite;
PImage[] textures = new PImage[4];

int strokeWidth = 20;
void setup(){
  size(1024,768, P3D);
  camera = new Camera();
  textures[0] = loadImage("mars.jpg");
  textures[1] = loadImage("neptune.png");
  textures[2] = loadImage("mercury.jpg");
  textures[3] = loadImage("jupiter.jpg");
  space_background = loadImage("space.jpg");
  asteroid = loadImage("asteroid.jpg");
  rocketship = loadImage("rocket.png");
  earth = loadImage("earth.jpg");
  satellite = loadImage("satellite.png");
  testPRM();
}

int numCollisions;
float pathLength;
boolean reachedGoal;

Vec3 sampleFreePos(){
  //Vec3 randPos = new Vec3(random(width),random(height),0);
  Vec3 randPos = new Vec3(random(100,900),random(100,650),0);
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  while (insideAnyCircle){
    //randPos = new Vec3(random(width),random(height),0);
    randPos = new Vec3(random(100,900),random(100,650),0);
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos);
  }
  return randPos;
}
void testPRM(){
  //long startTime, endTime;
  
  placeRandomObstacles(numObstacles);
  
  
  startPos = sampleFreePos();
  goalPos = sampleFreePos();
  //int start_index = (int)random(0, numNodes-2);
  //int goal_index = (int)random(0, numNodes-2);
  //startPos = nodePos[start_index];
  //goalPos = nodePos[goal_index];
  agentPos = startPos;
  curPathIdx = 0;

  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  
  //startTime = System.nanoTime();
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  //endTime = System.nanoTime();
  //pathQuality();
  
  //println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
  //        " Path Len:", pathLength, " Path Segment:", curPath.size()+1,  " Num Collisions:", numCollisions);
}

Vec3 computeAgentVel(){
  if(curPathIdx < curPath.size() - 1) {
     Vec3 curNodePos = new Vec3(nodePos[curPath.get(curPathIdx+1)].x, nodePos[curPath.get(curPathIdx+1)].y,0);
     Vec3 new_vel = curNodePos.minus(agentPos).normalized();
     // Vec3 new_vel = agentPos.minus(path[curPathIdx]).normalized();
     float dist = curNodePos.distanceTo(agentPos);
     hitInfo hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos, new_vel, dist);
     if (!hit.hit) {
       curPathIdx++;
     }
  }
  
  Vec3 new_vel = goalPos.minus(agentPos).normalized();
  float dist = goalPos.distanceTo(agentPos);
  hitInfo hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos, new_vel, dist);
  
  //Vec3 target;
  if (hit.hit) {
    target = new Vec3(nodePos[curPath.get(curPathIdx)].x, nodePos[curPath.get(curPathIdx)].y,0);
  } else {
    target = goalPos;
  }
  
  
  Vec3 agentVel = target.minus(agentPos).normalized().times(goalSpeed);
  if (agentPos.distanceTo(target) < 5){
    agentVel = new Vec3(0,0,0);
    //Vec3 newNodePos = new Vec3(nodePos[curPath.get(curPathIdx)].x, nodePos[curPath.get(curPathIdx)].y);
    //if(newNodePos.distanceTo(target) < 5){
    //  return new Vec3(0,0);
    //}
    if (target != goalPos) curPathIdx++;
  }
  return agentVel;
}

void moveAgent(float dt){
  agentVel = computeAgentVel();
  agentPos.add(agentVel.times(dt));
}
int rot_counter = 1;

void draw(){
  //println("FrameRate:",frameRate);
  pointLight(128, 128, 128,0,0,500);
  pointLight(128, 128, 128,0,0,500);
  pointLight(128, 128, 128,0,0,500);
  pointLight(128, 128, 128,0,0,500);
  strokeWeight(1);
  background(space_background);

  stroke(0,0,0);
  fill(255,255,255);
  
  camera.Update(1.0 / 25 * frameRate);
  
  moveAgent(1.0/frameRate);

  //Draw the planets
  for (int i = 1; i < numObstacles; i++){
    pushMatrix();
    float r = circleRad[i];
    noStroke();
    sphereDetail(8);
    globe = createShape(SPHERE, r-10);
    globe.setTexture(textures[circleTexture[i]]);
    noStroke();
    Vec3 c = circlePos[i];
    translate(c.x, c.y, c.z);
    shape(globe);
    popMatrix();
  }

  //Draw the asteroid, little special b/c the user controls it
  pushMatrix();
  float r = circleRad[0];
  noStroke();
  sphereDetail(8);
  globe = createShape(SPHERE, r-10);
  globe.setTexture(asteroid);
  noStroke();
  Vec3 c = circlePos[0];
  translate(c.x, c.y, c.z);
  shape(globe);
  popMatrix();

  //Draw Goal Planet Earth
  fill(255,255,255);
  pushMatrix();
  float g_rad = 60.0;
  noStroke();
  sphereDetail(8);
  globe = createShape(SPHERE, g_rad-10);
  globe.setTexture(earth);
  noStroke();
  Vec3 g_c = new Vec3(goalPos.x, goalPos.y ,0);
  translate(g_c.x, g_c.y, g_c.z);
  shape(globe);
  popMatrix();
  
  if (curPath.size() >0 && curPath.get(0) == -1) return; //No path found
  
  //Draw Rocket
  pushMatrix();
  translate(agentPos.x, agentPos.y, agentPos.z);
  fill(255,20,20);
  old_rotate = rotate;
  rotate = (float)Math.atan2(nodePos[curPath.get(curPathIdx)].y-agentPos.y, nodePos[curPath.get(curPathIdx)].x-agentPos.x);
  if(target == goalPos){
    rotate = (float)Math.atan2(goalPos.y-agentPos.y, goalPos.x-agentPos.x);
  }
  //***********************************************
  //if(abs(old_rotate-rotate) > 0.2){
  //  //int rot_counter = 1;
  //  //rotateZ(radians(90) + rot_counter*0.1);
  //  //rotate = rot_counter*0.1;
  //  //rot_counter += 1;
  //  if(rotate < 0){
  //    rotateZ(radians(90) - rot_counter*0.1);
  //    rotate = -rot_counter*0.1;
  //    rot_counter += 1;
  //  }
  //  else{
  //    rotateZ(radians(90) + rot_counter*0.1);
  //    rotate = rot_counter*0.1;
  //    rot_counter += 1;
  //  }
  //  print("inc");
  //}
  //else{
  //  rotateZ(radians(90)+rotate);
  //  rot_counter = 1;
  //  //old_rotate = rotate;
  //}
  //***************************************************
  rotateZ(radians(90)+rotate);
    //drawCylinder(20, 5, 1, 50);
  //}
  
  //rotateZ(radians(90)+rotate);
  //rotateZ(radians(90));
  drawRocket(20, 5, 1, 50);
  popMatrix();
  //curPath = planPath(agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  //curPathIdx = 0;
}

// Code from Jan Vantomme, I am just making use of it and added my own modifications
void drawRocket( int sides, float r1, float r2, float h)
{
    float angle = 360 / sides;
    float halfHeight = h / 2;

    // draw top of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r2;
        float y = sin( radians( i * angle ) ) * r2;
        //vertex( x, y, -halfHeight);
        vertex( x, -halfHeight,y);
    }
    endShape(CLOSE);

    // draw bottom of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r1;
        float y = sin( radians( i * angle ) ) * r1;
        //vertex( x, y, 0);
        vertex( x, 0, y);
    }
    endShape(CLOSE);
    
    // draw sides
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x1 = cos( radians( i * angle ) ) * r2;
        float y1 = sin( radians( i * angle ) ) * r2;
        float x2 = cos( radians( i * angle ) ) * r1;
        float y2 = sin( radians( i * angle ) ) * r1;
        //vertex( x1, y1, -halfHeight);
        //vertex( x2, y2, 0);
        vertex( x1, -halfHeight, y1);
        vertex( x2, 0, y2);    
    }
    endShape(CLOSE);
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r1;
        float y = sin( radians( i * angle ) ) * r1;
        //vertex( x, y, 0);
        vertex( x, 0, y);
    }
    endShape(CLOSE);

    // draw bottom of the tube
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r1;
        float y = sin( radians( i * angle ) ) * r1;
        //vertex( x, y, halfHeight);
        vertex( x, halfHeight,y);
    }
    endShape(CLOSE);
    
    // draw sides
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x = cos( radians( i * angle ) ) * r1;
        float y = sin( radians( i * angle ) ) * r1;
        //vertex( x, y, halfHeight);
        //vertex( x, y, 0);    
        vertex( x, halfHeight,y);
        vertex( x, 0, y);    
    }
    endShape(CLOSE);

}

boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    numObstacles = 10;
    generate_obstacle = false;
    testPRM();
    return;
  }
  if (key == 'o'){
    generate_obstacle = !generate_obstacle;
  }
  
  if (keyCode == SHIFT){
    shiftDown = true;
  }
  
  float speed = 10;
  if (shiftDown) speed = 30;
  if (keyCode == RIGHT){
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[0].x -= speed;
  }
  if (keyCode == UP){
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[0].y += speed;
  }
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  //curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPath = planPath(agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPathIdx = 0;
  camera.HandleKeyPressed();
  
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
  camera.HandleKeyReleased();
}

void mousePressed(){
  if (mouseButton == RIGHT){
    startPos = new Vec3(mouseX, mouseY,0);
    agentPos = startPos;
    //println("New Start is",startPos.x, startPos.y);
  }
  if (mouseButton == LEFT && generate_obstacle && numObstacles < maxNumObstacles){
    numObstacles += 1;
    circlePos[numObstacles-1] = new Vec3(mouseX, mouseY,0);
    circleRad[numObstacles-1] = (50+10*random(2));
    circleTexture[numObstacles-1] = (int)random(0,4);
  }
  if (mouseButton == LEFT && !generate_obstacle){
    goalPos = new Vec3(mouseX, mouseY,0);
    //println("New Goal is",goalPos.x, goalPos.y);
  }
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  //curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPath = planPath(agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPathIdx = 0;
}