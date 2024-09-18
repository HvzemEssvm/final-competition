import math

def calc_kick_pose(goal_pose, ball_pose): 
    """calculate the position of before kicking the ball"""
    # goal_pose => [x, y] : this point should be constant according to the map
    # ball_pose => [x, y] : we should get that point from the sensors

    kick_distance = 0.25 # the distance between the ball and the robot
    kick_pose = [0,0,0] # (x, y, theta)

    theta_rad = math.atan(((goal_pose[0] - ball_pose[0]) / (goal_pose[1] - ball_pose[1])))
    theta = theta_rad * (180 / math.pi)

    if theta < 0: # obtuse angle
        alpha = 180 - theta

        kick_pose[0] = round((ball_pose[0] + kick_distance * math.cos(alpha)), 2)
        kick_pose[1] = round((ball_pose[1] - kick_distance * math.sin(alpha)), 2)
        kick_pose[2] = round((math.pi / 2) - theta , 2)

    elif theta > 0 : # acute
        kick_pose[0] = round((ball_pose[0] - kick_distance * math.cos(theta_rad)), 2)
        kick_pose[1] = round((ball_pose[1] - kick_distance * math.sin(theta_rad)), 2)
        kick_pose[2] = round(90 - theta , 2)
    else:
        kick_pose[0] = 0
        kick_pose[1] = round(ball_pose[1] - kick_distance, 2)
        kick_pose[2] = 0
    return kick_pose


# examples:
print(calc_kick_pose([0,2], [0,1.4]))
print(calc_kick_pose([0,2], [2,1.2]))
print(calc_kick_pose([0,2], [0.5,0.4]))