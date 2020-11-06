import java.util.*;

//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec3[] centers, float[] radii, int numObstacles, Vec3[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec3 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      //float[] tempradii = new float[maxNumObstacles];
      //for (int rad = 0; rad < radii.length; rad++){
      //  tempradii[rad] = radii[rad];
      //}
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//Now returns closest node that does not result in a collison
int closestNode(Vec3 point, Vec3[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  hitInfo hit;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    //float segmentLength = nodePos[i].distanceTo(point);
    //Vec3 dir = point.minus(nodePos[i]).normalized();
    Vec3 dir = nodePos[i].minus(point).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, point, dir, dist);
    if (dist < minDist && !hit.hit){
      closestID = i;
      minDist = dist;
    }
  }
  if(closestID == -1){
    for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
      if (dist < minDist){
        closestID = i;
        minDist = dist;
      }
    }
    //print("There is not a valid stright line path from the startPos/GoalNode to a Node on the graph");
  }
  return closestID;
}

ArrayList<Integer> planPath(Vec3 startPos, Vec3 goalPos, Vec3[] centers, float[] radii, int numObstacles, Vec3[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestNode(startPos, nodePos, numNodes);
  int goalID = closestNode(goalPos, nodePos, numNodes);
  
  path = runAStar(nodePos, numNodes, startID, goalID);
  
  return path;
}

public class pQueue
{
  float priority;
  int index;

  public pQueue(float priority, int index)
  {
    this.priority = priority;
    this.index = index;
  }
  public pQueue()
  {
  }
}

public static Comparator<pQueue> priorityComparator = new Comparator<pQueue>(){
    
    @Override
    public int compare(pQueue p1, pQueue p2) {
            return (int) (p1.priority - p2.priority);
        }
  };
// Astar
ArrayList<Integer> runAStar(Vec3[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> path = new ArrayList();
  PriorityQueue<pQueue> priorityQueue = new PriorityQueue<pQueue>(priorityComparator);
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }
  //println("\nBeginning Search");
  if (startID != -1){
    visited[startID] = true;
  }
  int tempIndex = startID;
  pQueue tempPQueue = new pQueue(0.0, tempIndex);
  priorityQueue.add(tempPQueue);

  while (priorityQueue.size() > 0){
    if (startID == -1){
      priorityQueue.clear();
      break;
    }
    pQueue p_top = priorityQueue.poll();
    int currentNode = p_top.index;
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        
        if (goalID == -1){
          priorityQueue.clear();
          break;
        }
        float tempNewPriority = (p_top.priority + nodePos[currentNode].distanceTo(nodePos[neighborNode]))
                                + nodePos[neighborNode].distanceTo(nodePos[goalID]); //heuristic
        int tempNewIndex = neighborNode;
        pQueue tempNewPQueue = new pQueue(tempNewPriority, tempNewIndex);
        priorityQueue.add(tempNewPQueue);
      }
    } 
  }
  if (priorityQueue.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  
  return path;
}
