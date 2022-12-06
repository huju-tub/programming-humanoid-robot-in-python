'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
import os

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        ROBOT_POSE_CLF = 'robot_pose.pkl'
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(os.path.dirname(os.path.abspath(__file__)) + '\\' + ROBOT_POSE_CLF, 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        # YOUR CODE HERE
        postures = ["Back", "Belly", "Crouch", "Frog", "HeadBack", "Knee", "Left", "Right", "Sit", "Stand", "StandInit"]
        joints = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch"]        
        
        all_data = [perception.joint[joint] for joint in joints]
        all_data += (perception.imu[0], perception.imu[1])
        prediction = self.posture_classifier.predict([all_data])
        return postures[prediction[0]]

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
