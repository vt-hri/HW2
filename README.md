# HW2
starter code for HW2


- collect demos that pick up cube
- some more efficient than others, some fail, cube position changes
- save in folder with json, oracle score, and photos for replay
- initialize a reward model with output between 0 and 1

- sample a pair. then use their true scores to see how human chooses
- write a function to get total reward for a json (according to model)
- learn reward function so that sum across preferred is greater than across not


human teleoperates the robot arm / gather pairs of trajectories
break the scoring function into features and params
- cross entropy loss to figure out the params


features: take in trajectory, init state, map to a score

test: give it two new trajectories, see which one gets higher score


need: json to replay
need: library of traj pairs
need: human labels them
need: reward model
need: crossentropy loss, train



replay pairs of trajectories
features: intermediate point or not
features: red cube or gray cube
human picks across a few questions (0,1)
assign [-1, 1] for each features
then learn theta in [-1, 1] with metropolis hastings
or have a discrete set


save frames from keypoints along those trajectories
