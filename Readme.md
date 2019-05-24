# Group 3 RHBP workspace

This repository is based on the mapc_rhbp_example repo. 

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
4. Run the massim server and then run the package in a different terminal as 
   ```
   roslaunch rhbp_workspace rhbp_agents_all.launch
   ```
   
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
