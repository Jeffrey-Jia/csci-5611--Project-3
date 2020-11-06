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
int numNodes = 999;
int numAgents = 2;
  
//A list of circle obstacles
static int maxNumObstacles = 15;
Vec3 circlePos[] = new Vec3[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii
int circleTexture[] = new int[maxNumObstacles];
boolean generate_obstacle = false;
PShape globe;

static int maxNumAgents = 15;
Vec3 startPos[] = new Vec3[maxNumAgents];
//Vec3 startPos = new Vec3(100,500,0);
Vec3 goalPos = new Vec3(500,200,0);
Vec3 target[] = new Vec3[maxNumAgents];

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
Vec3 agentPos[] = new Vec3[maxNumAgents];
Vec3 agentVel[] = new Vec3[maxNumAgents];
//Vec3 agentPos = startPos;
//Vec3 agentVel = new Vec3(200,40,0);
float goalSpeed = 40;

//int curPathIdx = 0;
int curPathIdx[] = new int[maxNumAgents];
ArrayList<ArrayList<Integer>> curPath = new ArrayList<ArrayList<Integer>>(maxNumAgents);

ArrayList<Integer> tempPath;

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
  
  for (int i = 0; i < numAgents; i++){
    startPos[i] = sampleFreePos();
  }
  goalPos = sampleFreePos();
  //int start_index = (int)random(0, numNodes-2);
  //int goal_index = (int)random(0, numNodes-2);
  //startPos = nodePos[start_index];
  //goalPos = nodePos[goal_index];
  agentPos = startPos;
  for (int i = 0; i < numAgents; i++){
    curPathIdx[i] = 0;
  }

  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  
  //startTime = System.nanoTime();
  for (int i = 0; i < numAgents; i++){
    tempPath = planPath(startPos[i], goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
    curPath.add(tempPath);
  }
  //endTime = System.nanoTime();
  //pathQuality();
  
  //println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
  //        " Path Len:", pathLength, " Path Segment:", curPath.size()+1,  " Num Collisions:", numCollisions);
}

hitInfo hit[] = new hitInfo[maxNumAgents];

Vec3[] computeAgentVel(){
  for (int i = 0; i < numAgents; i++){
    if(curPathIdx[i] < curPath.get(i).size() - 1) {
       Vec3 curNodePos = new Vec3(nodePos[curPath.get(i).get(curPathIdx[i]+1)].x, nodePos[curPath.get(i).get(curPathIdx[i]+1)].y,0);
       Vec3 new_vel = curNodePos.minus(agentPos[i]).normalized();
       // Vec3 new_vel = agentPos.minus(path[curPathIdx]).normalized();
       float dist = curNodePos.distanceTo(agentPos[i]);
       hit[i] = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos[i], new_vel, dist);
       if (!hit[i].hit) {
         curPathIdx[i]++;
       }
    }
    
    Vec3 new_vel = goalPos.minus(agentPos[i]).normalized();
    float dist = goalPos.distanceTo(agentPos[i]);
    hit[i] = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos[i], new_vel, dist);
    
    //Vec3 target;
    if (hit[i].hit) {
      //target[i] = goalPos;
      if(curPathIdx[i] > curPath.get(i).size()){
        target[i] = goalPos;
      }
      target[i] = new Vec3(nodePos[curPath.get(i).get(curPathIdx[i])].x, nodePos[curPath.get(i).get(curPathIdx[i])].y,0);
    } else {
      target[i] = goalPos;
    }
    
    
    agentVel[i] = target[i].minus(agentPos[i]).normalized().times(goalSpeed);
    if (agentPos[i].distanceTo(target[i]) < 20){
      agentVel[i] = new Vec3(0,0,0);
      //Vec3 newNodePos = new Vec3(nodePos[curPath.get(curPathIdx)].x, nodePos[curPath.get(curPathIdx)].y);
      //if(newNodePos.distanceTo(target) < 5){
      //  return new Vec3(0,0);
      //}
      if (target[i] != goalPos) curPathIdx[i]++;
    }
  }
  ////Avoidance Force
  //for (int i = 0; i < numAgents; i++){
  //  for (int j1 = i; i > 0; i--){
  //    if (j1 == i){
  //      agentVel[i] = agentVel[i];
  //    } else {
  //      if(agentPos[i].distanceTo(agentPos[j1]) < 40){
  //        if(agentPos[i].distanceTo(goalPos) < agentPos[j1].distanceTo(goalPos)){
  //          agentVel[i] = new Vec3(agentVel[i].x*2,agentVel[i].y*2,0);
  //          agentVel[j1] = new Vec3(0,0,0);
  //        } else {
  //          agentVel[j1] = new Vec3(agentVel[j1].x*2,agentVel[j1].y*2,0);
  //          agentVel[i] = new Vec3(0,0,0);
  //        }
  //      }
  //    }
  //  }
  //  for (int j2 = i + 1; i < numAgents; i++){
  //    if(agentPos[i].distanceTo(agentPos[j2]) < 40){
  //      if(agentPos[i].distanceTo(goalPos) < agentPos[j2].distanceTo(goalPos)){
  //          agentVel[i] = new Vec3(agentVel[i].x*2,agentVel[i].y*2,0);
  //          agentVel[j2] = new Vec3(0,0,0);
  //        } else {
  //          agentVel[j2] = new Vec3(agentVel[j2].x*2,agentVel[j2].y*2,0);
  //          agentVel[i] = new Vec3(0,0,0);
  //        }
  //    }
  //  }
  //}
  return agentVel;
}

void moveAgent(float dt){
  agentVel = computeAgentVel();
  for (int i = 0; i < numAgents; i++){
    agentPos[i].add(agentVel[i].times(dt));
  }
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
    globe = createShape(SPHERE, r-15);
    globe.setTexture(textures[circleTexture[i]]);
    noStroke();
    Vec3 c = circlePos[i];
    translate(c.x, c.y, c.z);
    shape(globe);
    popMatrix();
  }

  ////Draw the first circle a little special b/c the user controls it
  pushMatrix();
  float r = circleRad[0];
  noStroke();
  sphereDetail(8);
  globe = createShape(SPHERE, r-15);
  globe.setTexture(asteroid);
  noStroke();
  Vec3 c = circlePos[0];
  translate(c.x, c.y, c.z);
  shape(globe);
  popMatrix();

  //Draw Goal
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
  
  for (int i = 0; i < numAgents; i++){
    if (curPath.get(i).size() >0 && curPath.get(i).get(0) == -1) return; //No path found
    
    //Draw Rocket
    pushMatrix();
    translate(agentPos[i].x, agentPos[i].y, agentPos[i].z);
    fill(255,20,20);
    old_rotate = rotate;
    rotate = (float)Math.atan2(nodePos[curPath.get(i).get(curPathIdx[i])].y-agentPos[i].y, nodePos[curPath.get(i).get(curPathIdx[i])].x-agentPos[i].x);
    if(target[i] == goalPos){
      rotate = (float)Math.atan2(goalPos.y-agentPos[i].y, goalPos.x-agentPos[i].x);
    }
    //float rotate = (float)Math.atan2(agentPos.x-nodePos[curPath.get(curPathIdx)].x, agentPos.y-nodePos[curPath.get(curPathIdx)].y);
    //println(abs(old_rotate-rotate));
    //println(frameRate);
    //if(abs(old_rotate-rotate) > 0.5){
    //  for(int inc = 0; inc < 4; inc++){
    //      rotateZ((radians(90)+rotate)/4);
    //      pushMatrix();
    //      drawCylinder(20, 5, 1, 50);
    //      popMatrix();
    //      delay(50);
    //  }
    //}
    //else{
    //  rotateZ((radians(90)+rotate));
    //  drawCylinder(20, 5, 1, 50);
    //}
    //if(abs(old_rotate-rotate) > 0.2){
    //  //int rot_counter = 1;
    //  //rotateZ(radians(90) + rot_counter*0.1);
    //  //rotate = rot_counter*0.1;
    //  //rot_counter += 1;
    //  if(rotate < 0){
    //    rotateZ(radians(-90) - rot_counter*0.1);
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
    rotateZ(radians(90)+rotate);
    drawRocket(20, 5, 1, 50);
    popMatrix();
    //curPath = planPath(agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
    //curPathIdx = 0;
  }
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
  //curPath.clear();
  for (int i = 0; i < numAgents; i++){
    tempPath = planPath(agentPos[i], goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
    curPath.set(i,tempPath);
    curPathIdx[i] = 0;
  }
  
  camera.HandleKeyPressed();
  
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
  camera.HandleKeyReleased();
}

void mousePressed(){
  //if (mouseButton == RIGHT){
  //  //startPos = new Vec3(mouseX, mouseY,0);
  //  agentPos = startPos;
  //  //println("New Start is",startPos.x, startPos.y);
  //}
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
   //curPath.clear();
  for (int i = 0; i < numAgents; i++){
    tempPath = planPath(agentPos[i], goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
    curPath.set(i,tempPath);
    curPathIdx[i] = 0;
  }
}
