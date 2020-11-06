# CSCI 5611 - Project 3: AI Planning
### Jeffrey Jia | jiaxx215

## Controls

Camera:<br>
    - TFGH - Move the camera up, down, left, right on the xy-plane<br>
    - A/D - Rotate the camera counterclockwise/clockwise about the y-axis<br>
    - W/S - Rotate the camera up/down about the x-axis<br>
    - Q/E - Move camera in/out of the xy plane <br>
    - Z - Reset camera position

User Controlled Obstacle (Asteroid):
    - Arrow Keys - Move the Asteroid up, down, left, right on the xy-plane<br>

Obstacle Generation (Planets):
    - O - Toggle obstacle generation on/off<br>

Mouse:
    - Left Click (while obstacle generation toggled off) - Move goal to click location (earth)
    - Left Click (while obstacle generation toggled on) - Generate planet obstacle at click location  

**NOTE: The camera's cframe is independent of the Asteroid's cframe, vice versa. What this means is that if the camera is moved in some arbitrary fashion and you start to move the Asteroid, the Asteroid would move in respect to the world and not the camera (e.g. left arrow key to move left means the Asteroid will move left in respect to the world, but would probably move "right" in camera view)**

Simulation:
    - G - Pause/Unpause simulation


  ## Implementation features

  **A quick note: Our cloth is based on a mixture of polyester-like and feathersilk-like materials, which results in a cloth that is somewhat airy**


  - Single Agent Navigation* (50)
      - I used my hw3 implementation of an A* PRM that allows a single agent, in this case a rocket, navigate through space
      whilst avoiding obstacles, in this case planets and an user controlled asteroid.

  - 3D Rendering & Camera (10)
      - The planets, asteroid, rocket, and camera exist in 3D space, and is able to render at ~45 FPS. There are also directional and point lights shot onto the scene to provide the planets with a more 3D look. For asteroid and camera controls, see above.

  - Improved Agent & Scene Rendering (10)
      - The agent is a cone shape stacked on top of a cylinder that I rendered using custom vertices. This shape is not a basic shape
      nor spherical which means rotation plays a part in obstacle avoidance.
      - The obstacles are also draw in 3d and textured (both the planets and the asteroid).

  - Orientation Smoothing (10)
      - As the rocket changes direction, the direction the tip of the rocket faces is the same as the direction the rocket is traveling. This change occurs gradually.

  - Planning Rotation (10)
      - There are times when my agent, the rocket, can only fit through a passage if it is oriented in the correct fashion.
          - Since I can decide the locations of my obstacles, I put two near each other with a small gap where the longest dimension
          would not fit to see if my agent(s) would pass through it properly, which it did.

  - User Scenario Editing+ (10)
      - The user can control the Asteroid, which is an obstacle, using the controls mentioned above.
      - The user can also spawn new planets if they click 'o' to toggle obstacle generation and then left click anywhere on the window.

  - Realtime User Interaction (10)
      -  The real time movement of the Asteroid changes the path of the agent(s).
      -  The real time spawning of new Planets changes the path of the agent(s).

  - Multiple Agents Planning+ (10)
      - My implementation allows for the creation of multiple agents, in my case rockets.
          - Each of these rockets spawn in a random location and have their own best path to the goal (Earth).

  - Crowd Simulation+ (20)
     - Not Implemented

  ## Tools Used

  - Processing
  - Java programming language


  ## Difficulties encountered

  I could not get the Crowd simulation element working. I think this is likely due to the way my code was laid out prior to attempting to account for crowd simulation.

  The Orientation Smoothing took a long time to get to work, it was very difficult for me to make the agent not simply snap in place.

  ## Video

  [Video Link](https://youtu.be/w036hNs8SpU)

  Timestamps: <br>
  - 0:08 - Starting the simulation<br>
  - 0:08 - 0:30 - Camera movement<br>
  - 0:33 - 1:00 - Cloth and ball movement through user input (left click and arrow keys, respectively)<br>
  - 1:00 - 2:13 - Ball and cloth interaction. Showing as to how air drag affects the cloth as it falls down and how realistic it is to cloth you see in real life.
  - 2:28 - 2:55 - User interaction with the cloth with mouse input. Also shows as to how realistic the cloth simulation is like.

  ## Art contest submission
  ***There is no art submission link***
  [Art Submission Link](https://imgur.com/)
