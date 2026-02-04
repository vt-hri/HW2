import numpy as np
import json

# load a demonstration
# there are 10 demonstrations (numbered 1 - 10) saved to the demos folder
def get_demo(demo_number):
    demo_name = "demos/demo" + str(demo_number) + ".json"
    with open(demo_name, 'r') as f:
        data = json.load(f)
    return data["features"]

# score a demonstration for given parameters theta
def score_demo(demo_number, theta):
    theta = np.array(theta)
    features = get_demo(demo_number)
    score = 0.0
    # sum score for each recorded feature vector
    for feature in features:
        score += feature @ theta
    return score

# human model [to complete]
def human_model(demo_number1, demo_number2, theta, beta=1.0):
    return None

# example code to get a demo features and score them
theta = [-1.0, 0, 0, 1.0]
demo = get_demo(1)
score = score_demo(1, theta)
print("demonstration features:\n", np.round(demo, 3))
print("score:", np.round(score, 3))
