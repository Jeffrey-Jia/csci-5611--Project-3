# CSCI 5611 - Project 3: AI Planning
### Jeffrey Jia | jiaxx215

## Controls

Camera:<br>
    - TFGH - Move the camera up, down, left, right on the xy-plane<br>
    - A/D - Rotate the camera counterclockwise/clockwise about the y-axis<br>
    - W/S - Rotate the camera up/down about the x-axis<br>
    - Q/E - Move camera in/out of the xy plane <br>
    - Z - Reset camera position<br>

User Controlled Obstacle (Asteroid):<br>
    - Arrow Keys - Move the Asteroid up, down, left, right on the xy-plane<br>

Obstacle Generation (Planets):<br>
    - O - Toggle obstacle generation on/off<br>

Mouse:<br>
    - Left Click (while obstacle generation toggled off) - Move goal to click location (earth) <br>
    - Left Click (while obstacle generation toggled on) - Generate planet obstacle at click location <br>

**NOTE: The camera's cframe is independent of the Asteroid's cframe, vice versa. What this means is that if the camera is moved in some arbitrary fashion and you start to move the Asteroid, the Asteroid would move in respect to the world and not the camera (e.g. left arrow key to move left means the Asteroid will move left in respect to the world, but would probably move "right" in camera view)**

Simulation:
    - R - Rerun simulation


  ## Implementation features

  - Single Agent Navigation* (50)
      - I used my hw3 implementation of an A* PRM that allows a single agent, in this case a rocket ship, navigate through space
      whilst avoiding obstacles, in this case planets and an user controlled asteroid.

  - 3D Rendering & Camera (10)
      - The planets, asteroid, rocket ship, and camera exist in 3D space, and is able to render at ~45 FPS. There are also directional and point lights shot onto the scene to provide the planets with a more 3D look. For asteroid and camera controls, see above.

  - Improved Agent & Scene Rendering (10)
      - The agent is a cone shape stacked on top of a cylinder that I rendered using custom vertices. This shape is not a basic shape
      nor spherical which means rotation plays a part in obstacle avoidance.
      - The obstacles are also draw in 3d and textured (both the planets and the asteroid).

  - Orientation Smoothing (10)
      - As the rocket ship changes direction, the direction the tip of the rocket ship faces is the same as the direction the rocket ship is traveling. This change occurs gradually.

  - Planning Rotation (10)
      - There are times when my agent, the rocket ship, can only fit through a passage if it is oriented in the correct fashion.
          - Since I can decide the locations of my obstacles, I put two near each other with a small gap where the longest dimension
          would not fit to see if my agent(s) would pass through it properly, which it did.

  - User Scenario Editing+ (10)
      - The user can control the Asteroid, which is an obstacle, using the controls mentioned above.
      - The user can also spawn new planets if they click 'o' to toggle obstacle generation and then left click anywhere on the window.

  - Realtime User Interaction (10)
      -  The real time movement of the Asteroid changes the path of the agent(s).
      -  The real time spawning of new Planets changes the path of the agent(s).

  - Multiple Agents Planning+ (10)
      - My implementation allows for the creation of multiple agents, in my case rocket ships.
          - Each of these rocket ships spawn in a random location and have their own best path to the goal (Earth).

  - Crowd Simulation+ (20)
     - The rocket ships are able to detect when other rocket ships are nearby and if it detects any other rocket ship are nearby it will stop its motion temporary until the other rocket ships pass.
          - When all the rocket ships start at the same location and need to move to the new goal location, they will line up and travel there in a single file line.

  **A quick note: Small bug where sometimes a rocket ship will spin a lot introduced by an issue with Orientation Smoothing, better than having the rocket only snap into orientation**

  ## Tools Used

  - Processing
  - Java programming language


  ## Difficulties encountered

  I could not quite get the Orientation Smoothing part working perfectly. When the agents rotate they definatly do not snap into pace and it is clear they gradually turn. However, it is sometimes the case where some of the agent over rotate.  

  I could not get the Crowd simulation element working as well as I would have liked, but I was still able to get interaction among the agents. I think this is likely due to the way my code was laid out prior to attempting to account for crowd simulation.


  ## Video

  [Video Link](https://www.youtube.com/watch?v=6Bt4HjPZEmU&feature=youtu.be&hd=1)

  Timestamps: <br>
  - 0:03 - Starting the simulation<br>
  - 0:03 - 0:24 - Single Agent Navigation<br>
  - 0:24 - 0:50 - Camera movement<br>
  - 0:50 - 1:10 - Improved Agent & Scene Rendering<br>
  - 1:18 - 1:55 - User Scenario Editing via translating the Asteroid and the addition of Planets<br>
  - 1:18 - 1:55 - Realtime User Interaction as the Agent's path changes as new obstacles are introduced<br>
  - 1:45 - 1:52 - Planning Rotation as rocket ship flies through a space that its longest dimension would not<br>
  - 2:20 - 3:30 - Multiple Agents Planning as shown with 10 agents: each follow their own best path<br>
  - 2:20 - 3:30 - Crowd Simulation <br>
      - 1. 2:27-2:37 and 3:00-3:10 - Rocket ship stop when there are other rocket ships nearby (ignore spinning one, bug introduced by small bug with Orientation Smoothing)<br>
      - 2. 2:45-3:00 - When all the rocket ships start at the same location and need to move to the new goal location, they will line up and travel there in a single file line.<br>

  ## Art contest submission
  ***There is no art submission link***
  [Art Submission Link](https://imgur.com/)
