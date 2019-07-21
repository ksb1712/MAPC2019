# Group 3 RHBP workspace

This repository is based on the mapc_rhbp_example repo. 
Requirements:
python 2.7
numpy
matplotlib

Usage:

1. Setup mapc_workspace as instructed in <a href="https://gitlab.tubit.tu-berlin.de/aaip-ss19/mapc_workspace">this</a> repository
2. clone this repo as <b>rhbp_workspace</b> into the src directory
   ``` 
   cd mapc_workspace/src
   git clone https://gitlab.tubit.tu-berlin.de/aaip-ss19/group3 rhbp_workspace
   ```
3. run catkin_make
   ```
   cd ../
   catkin_make
   source devel/setup.bash
   ```
4. Navigate to src folder in rhbp_workspace and  run the massim server and then run the package in a different terminal as 
   ```
   roslaunch rhbp_workspace rhbp_agents_all.launch
   ```
5. To run map, navigate to folder where roslaunch runs (will be displayed on screen) /map
   ```
   python plot.py
   ```
# Server setup 

### For quick visualisation, it is better to have only one team in the map.

1. Go to server config:
    ```
    cd mapc_workspace/third-party/massim/server/conf
    ```
    
2. Change SampleConfig2.json:
    
    ```
    "teams" : {
        "A" : "$(teams/A.json)",
        "B" : "$(teams/B.json)"
    }
    ```
    to
    
    ```
     "teams" : {
        "A" : "$(teams/A.json)"
        }
    ```

3. And in server/server.json change teamsPerMatch to
    
    ```
    "teamsPerMatch" : 1,
    ```
    

### To have only a single agent per team 
    
1. In SampleCongif2.json change the entities line to:
    ```
    "entities" : [{"standard" : 1}],
    ```
2. And comment out the laucher for agents 2 - 10 in rhbp_workspace/launch/rhbp_agents_full.launch

# Contibuting
To include your changes to the repo, create a local branch with your name 

```
cd mapc_workspace/src/rhbp_workspace
git checkout -b <Name>
git commit -m"Message"
git push origin <Name>
```
Follow <a href="https://chris.beams.io/posts/git-commit/">this</a> convention for commit messages.
<br>
Write the summary line and description of what you have done in the imperative mood, 
<br>that is as if you were commanding someone. Start the line with "Fix", "Add", "Change" <br> 
instead of "Fixed", "Added", "Changed".
