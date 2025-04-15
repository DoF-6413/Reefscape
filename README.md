# Reefscape
Code base for FRC Team 6413's 2025 Reefscape Robot Code

## Code Etiquette
  1. Comments on EVERYTHING (Commands, Constants, etc) <br/>
  2. Organize files properly in corresponding folders <br/>
  3. Follow DoF [Naming Conventions](README.md#naming-conventions) <br/>
  4. Create Issues on Github for EVERY branch
     - add a description of what the branch should accomplish
     - 'Assignees': used to indicate who is working on what
     - 'Labels': used to organize issues, add as necessary
     - 'Development': link the branch to the issue
     
  6. Sign in, sign out!
     - Use your designated folder and ssh key
     - Log out of Slack, Github, Google, and anything else at the end of the day

  8. Version Number
     - first number = Number of pushes (commits) to Dev
     - second number = Issue number
     - third number = Commit number (of this branch)
     - fourth number = Functionality: 0 = working, 1 = WIP, 2 = doesn't work 

For example, with 6 commits to Dev, on the branch associated with issue 4 with 13 commits, which is completely functional (tested for consistency) should look like: "6.4.13.0"

## Branch Organization

main:

- Competition-ready code
- ONLY receives pushes from Dev
- Do *NOT* make branches off of main

Dev:

- Used for full systems tests
- Only push to main after tested and cleaned
  - No hardcoded values, add them to Constants
  - Explanatory comments
  - Code is fully functional
- Make ALL branches off of *this* branch!!

feat#[Issue#]-[name]:

- Used for new features on the robot
  - Ex: feat#40-arm
- Used for code changes
  - Changes can be as small or big as necessary
    - Ex: feat#68-mechanism-button-bindings
  - Only contains what the branch is named
    - Ex: No climber code in a shooter branch
- Branch name is all lowercase and hyphenated
  - Ex: feat#69-shooter-interpolation, not feat#69-ShooterInterpolation
-  Push to Dev when code is tested and clean (see "Dev" for "tested and clean" standards)

bugfix#[Issue#]-[name]:

- Used for bugfixing of a feature from Dev
- Branch name is all lowercase and hyphenated

chore#[Issue#]-[name]:

- Used for cleaning code from Dev
- Branch name is all lowercase and hyphenated

## Naming Conventions
- Folder and file names
  - CapitalizeEveryWordWithoutSpaces
  - *unless it's a subsystem in Subsystems, which are all lowercase (ex: "arm", "utbintake")
- Constants
  - ALL_CAPS_WITH_UNDERSCORES
- Functions
  - camelCase (lowercase first word, capitalize first letter of all subsequent words, no spaces)
- Class variables
  - All class variables should begin with "m_" to denote they are a "member" of said class

## Folder/File Organization
- **Commands**
  - **Auto Commands**
    - All of DoF's custom-built autonomous routines
  - **Drive Commands**
    - Includes different driving modes such as field-relative (default), robot-relative (used for certain autonomous routines), heading-locked, and automated routines we utilize such as feedforward and wheel radius characterizations.
  - **Pathfinding Commands**
    - The commands for on-the-fly trajectory following using PathPlanner's Pathfinding feature.
  - **Superstructure Commands**
    - Determines the state of the entire superstructure, or all the mechanisms not related to the drivetrain, typically including the elevator and end effectors

- **Subsystems**
  - For 2025, DoF's official subsystems are denoted as follows:
    - Periscope (PS) aka elevator
    - ALGAE End Effector (AEE)
    - CORAL End Effector (CEE)
    - Funnel (FL)
    - Climber (CL)
  - The code for a typical subsystem is organized by 6328's AdvantageKit framework, as follows: 
  - [subsystem]
    - [Subsystem.java]: main class for subsystem, runs commands depending on passed in IO (sim or real), extends SubsystemBase
    - [Subsystem]Constants.java: subsystem-specific constants
    - [Subsystem]IO.java: interface for inputs and methods
    - [Subsystem]IOSim.java: simulation code for subsystem, implements [Subsystem]IO
    - [Subsystem]IO[Motor].java: real code for subsystem, implements [Subsystem]IO
  - The Drive subsystem encompasses the Gyro, Odometry, and Pose Estimation utilities.

- **Utils**
- Other (not in a folder)
  - CAN IDs
    - A helpful reference of all the CAN IDs of the robot's electronics
  - Elastic
    - Configures Elastic dashboard layout
  - Constants
    - Constants shared by all subsystems, related to the robot
    - Ex: controller port numbers, battery voltage, alliance (red/blue), etc.
  - Main
    - Runs Robot
  - Robot
    - Initializes robot, starts logging data, runs periodic/auto/simulation modes
  - Robot Container
    - Creates all subsystems, runs commands, and logs data

## Useful Git Bash Commands
- git add .
  - Stages all code to prepare for commit and push
- git commit -m "[Insert Message Here]"
  - Saves code locally with a message of what was done to the code
- git push
  - Pushes code to the cloud
- git fetch
  - Tells the laptop there are new changes
- git pull
  - Puts new changes on laptop
  - git pull does git fetch
- git pull origin Dev
  - Pulls changes from Dev
- git merge (origin) Dev
  - Pulls ALL changes from Dev (no rebases)
- git checkout [branch name]
  - Changes your branch
- git checkout -b [branch name]
  - Creates new branch off of current branch
- git branch
  - Shows all branches that have been pulled on laptop
- git branch --all
  - Shows all branches since last fetch
- git config user.email [User Email]
  - Configures email for THAT specfic Folder
  - DO NOT USE GLOBAL ON SHARED LAPTOPS
  - Used to properly label who wrote what code through commits (not Access or PRs)
- git config user.name [Github Username]
  - Configures username for THAT Specific Folder
  - DO NOT USE GLOBAL ON SHARED LAPTOPS
  - Used to properly label who wrote what code through commits (Not Access or PRs)