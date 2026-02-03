# HW2
starter code for HW2


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
