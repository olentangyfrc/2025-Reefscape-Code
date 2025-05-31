<p align="center">
  <img src="https://team4611.wordpress.com/wp-content/uploads/2022/09/main_col.png?w=1024" alt="Ozone Robotics Logo" height="120">
</p>

<h1 align="center">Ozone Robotics (4611) - 2025 Reefscape Code</h1>

<p align="center">
  <a href="https://www.python.org/">
    <img src="https://img.shields.io/badge/Made%20with-Python-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python Badge">
  </a>
  <a href="https://ozonerobotics.org">
    <img src="https://img.shields.io/badge/Visit-ozonerobotics.org-0A192F?style=for-the-badge&logo=google-chrome&logoColor=white" alt="Website Badge">
  </a>
  <a href="https://www.paypal.com/donate/?hosted_button_id=MYQYBLG72HFN6">
    <img src="https://img.shields.io/badge/Donate-PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="Sponsor Badge">
  </a>
</p>

---

## 📖 Chapters
1. [Introduction](README.md#1--Introduction)
2. [Subsystems](README.md#2-%EF%B8%8F-subsystems)
3. [States](README.md#3--States)
4. [Vision](README.md#4--vision-processing-and-localization)
5. [Autonomous](README.md#5--autonomous)
6. [Logging](README.md#6-logging)
7. [Simulation](README.md#7-simulation)
8. [Utilities](README.md#8--utilities)
9. [Contributors](README.md#9--contributors)
---

### 1. 🧠 Introduction
Welcome to Ozone Robotics' 2025 code release for our robot Trident. This was the first year we switched to coding in Python using [RobotPy](https://robotpy.readthedocs.io/en/stable/index.html).

##### Notable Accomplishments:
- **Finger Lakes Reigonal**: Alliance 2 First Pick and Regional Winner
- **Buckeye Reigonal**: Alliance 1 Captain, Finalist, and Autonomous Award Winner
- **Curie Division**: Alliance 6 Captain and Semifinalist

We switched to python this year mainly due to its simpler syntax and greater ubiquity among newer students. Most of our new programmers learned Python as one of their first coding languages and we hope switching to Python can let us teach them more robotics-based things without having to worry about teaching them the coding language.

We use the [MagicBot Framework](https://robotpy.readthedocs.io/en/latest/frameworks/magicbot.html) which is a pythonic alternative to the more traditional [Commands Based Framework](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html). Essentially, instead of seperate command and subsystem files, everything is combined into what are called components. These components are then controlled in the main `robot.py` file.

##### Code Structure
- `robot.py`: Runs higher level logic for multiple components
- `components/`: Folder containing individual components (eg. Shooter, Elevator, etc.)
- `autonomous/`: Folder containing autonomous routines
- `utilities/`: Folder containing utility functions and constants

### 2. ⚙️ Subsystems
The robot is split into 5 main mechanical subsystems:
- Swerve drivetrain
- Cascade elevator 
- End Effector
- Algae Arm
- Ratchet Strap Climber
> **TODO:** Add CAD links if we decide to release.

Each of these subsystems has its own file within the code, defining it as a magic component. All components are initialized in `robot.py` which updates the states of every component, centralizing all logic.

### 3. 🔄 States
Instead of using the MagicBot's `StateMachine` class, we opted for our own custom state logic this season. All state variables are stored in a enum, those states are then incorporated into the `execute` method in each component, which runs the logic to swap between states.

> **TODO**: add example component (doesn't have to be detailed)

We opted for state-based control for three main reasons:
1. It allows for smooth, chronological transitions between actions
2. It allows for unified code structure within the team
3. It is more readable than command-based and fits into our framework

In addition to component state logic, we also implement global state machine logic in `robot.py`. Essentially, we modify each component's state in different robot states which we can then bind to either a button or autonomously run. This minimizes the amount of states that we must create because we can reuse a single state for multiple different actions (For example our auto-align drivetrain state is used in L1-L4 scoring as well as Net Scoring)

We opted against magicbot's pre-existing `StateMachine` class because it required at least two components to be implemented effectively, and it can get confusing defining extra components when we could reuse the same ones. **We would still recommend checking it out [here](https://robotpy.readthedocs.io/projects/utilities/en/latest/magicbot.html#module-magicbot.state_machine).**

### 4. 🎯 Vision Processing and localization
> **TODO**: maybe shorten this section and add things into drop downs.

For vision processing, we use two Limelight 3G cameras mounted above the front left and right swerve modules, each angled inward by approximately 20 degrees to maximize AprilTag visibility.

We use [Megatag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2) calculations, which take in the robot's gyro yaw value to figure out the robot's pose.

- `vision.py` (in the `components` folder) initializes all our Limelights and adds measurements to the drivetrain's pose estimator.
- To get measurements, we call `set_robot_orientation()` and `get_recent_vision_measurements()` from `vision_utils.py` (in the `utilities` folder).
- `vision_utils.py` is a Python adaptation of Java’s [Limelight Helpers](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib). (Note: not all methods are included, and standard deviation logic isn't fully generalizable.)
- `set_robot_orientation()` updates NetworkTables with the gyro yaw for Megatag2.
- `get_recent_vision_measurements()` returns all recent measurements since the last call (it discards measurements if there are more than 10 stored to reduce backlogged measurements from intialization

Most of our custom implementation is generalizable to any limelight-based vision subsystem. However certain things like standard deviations and rotational scaling is not and must be determined for your own robot. (Use the generic tag distance if you don't want to go through the hassle)

After recieving input from the cameras, we pass it into a wpilib `SwerveDrive4PoseEstimator` which is a kalman filter that fuses odometry data with vision measurements. This year we decided to lower the vision standard deviation measurements a lot because we found that our vision was much more accurate than our odometry when lining up.

**We run all of our vision processing as if we are on the blue alliance**. This allows us to not have to worry about different coordinates for each side and improves our match consistency.

### 5. 🤖 Autonomous
We use [choreo](https://choreo.autos/) to generate autonomous paths for our 3.5 piece and push auto and magicbot's `AutonomousStateMachine` class to run our own custom autonomous following logic from them.

All of our choreo paths are loaded on initialization, so to save space in memory most of our simpler autos (1pc and 2 net + dealgy) run using our pid line-up function instead.

Our autonomous routine is structured similarily to the rest of the codebase using state-based programming. But instead of an enum to store states, each state is its own function and states are swapped to whenever a certain condition is met.

**We select autos using a custom dasboard, not a sendable chooser**. This gives us both preset routines and allows us to create custom, on-the-fly paths by selecting reef nodes.

> **TODO**: add dropdown sections for specfic autos. Also add links to honeycomb if we decide to release

> **Note**: while magicbot natively supports a sendable chooser to select routines. We found it to be buggy and it's exclusive to SmartDasboard

### 6. Logging
We log data using magicbot's built in `feedback` tags. Wrapping a getter method with this logs all data produced by it to network tables in all modes. We can then use this data and display it or analyze it with [Advantage Scope](https://docs.advantagescope.org/)

> **Todo**: maybe add a example method wrapped with tag

For displaying data we use the [elastic dashboard](https://frc-elastic.gitbook.io/docs) which provides us a clean display to use during competition. We also use remote downloading so in our `deploy` folder there is a file named `ozone-elastic-layout.json` which we deploy to our robot and can download from there.

> **Todo**: add pictures of our competition elastic displays
> **Note**: We added a ton of things to our displays this year because both of our drivers were programmers. you typically do not need to add this much info.

### 7. Simulation
For simulation purposes we do not directly simulate each motor on the robot or use a `physics.py` file for [RobotPy physics support](https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html).

Instead we use a more basic alternative where we store each thing we wish to simulate as a seperate variable and modify those directly when in simulation mode. Overall this approach was much quicker to implement mid-season and gives us a good-enough testing ground for ideas. 

After simulating, we can view the position of our robot in either elastic or Advantage Scope using their built-in fields. Also we simulate the rest of our components, using boolean boxes to see if they reached a certain point, to better run autonomous routines and see how changes would affect certain actions like intaking or scoring.


> **TODO**: Maybe include a picture or video of this happening, this one is very optional however. Also we need to merge sim changes into main

### 8. 🛠 Utilities
Our `utilities` folder contains 6 files for general purposes throughout the code
- `constants.py`: A file containing constants for CAN id's, game obstacle positions, and line-up positions
- `ozone_utility_functions.py`: A file for utility functions that are not game-specific such as filtering, unit-conversion, and tolerance checking
- `file_utils.py`: A file exclusively for handling the `ozone-elastic-layout.json` file for remote downloading. It does some quality of life stuff like automatically updating it whenever we change the layout
- `reefscape_functions.py`: A file for functions exclusive to reefscape like getting target reef or algae points
- `vision_utils.py`: A file containing our modified adaptation of [Limelight Helpers](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib) which allow us to use Megatag 2 april tag detection. 
- `elasticlib.py`: this is taken directly from the [elastic github page](https://github.com/Gold872/elastic-dashboard) and allow for easy integration between code and the elastic dashboard such as automatic tab switching between autonomous and teleop and the ability to send notifications

### 9. 👥 Contributors

##### Core Contributors:
-  **Ethan Grieshop**: EthanGrieshop
-  **Neil Julian**: kracken6291
-  **Akshaj Katkuri**: Akshaj-Katkuri
-  **Anirudh Paladugula**: PRODOFFICIAL

##### Other Contributors:
- **Bennett Singer**: BennettSinger
- **Pranathi Irrinki**: Pranathi I
- **Sahil Gandhi**: Sahilg93
- **Shaun Thomas**: shaunT-08
- **Varun Nandakumar**: Varun N
- **Vivaan Singh**: vivaannotvivian
- **Wafee Qazi**: WafeeQazi
- **Zehra Demirtoka** zherapnda

##### Programming Mentors:
- **Jason Zutterling**: JasonZutterling
- **Jeff Brusoe**: jbfrc
---

### 🧡 Thank You To Our Sponsors!
>**TODO**: Makes sure to add pictures for **all** of our sponsors with larger ones going up top. All images should contain a link to their website.
(Sponsor logos or links could go here!)

![output-onlinepngtools](https://github.com/user-attachments/assets/e3176bfd-30f2-422c-ac45-c0d2c74c70a8)

