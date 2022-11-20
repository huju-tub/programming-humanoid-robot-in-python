'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello

def bezier(time_i, t, key):
    key_0 = key[time_i][0]
    key_3 = key[time_i+1][0]
    key_1 = key_0 + key[time_i][1][2]
    key_2 = key_3 + key[time_i][2][2]
    return (
        (1 -t) ** 3 * key_0
        + 3 * t * (1 - t) ** 2 * key_1
        + 3 * t**2 * (1 - t) * key_2
        + t**3 * key_3
    )
    
class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        if(self.keyframes == ([],[],[])):
            return target_joints
        
        if(self.start == None):
            self.start = perception.time
        time_diff = perception.time - self.start
 
        
        names, times, keys = keyframes
        n = len(names)
        
        for joint_i in range(n):
            name = names[joint_i]

            if not (name in self.joint_names):
                continue
            joint_times = times[joint_i]
            joint_keys = keys[joint_i]
                  
            for time_i in range(len(joint_times) - 1):
                joint_time_0 = joint_times[time_i]
                joint_time_1 = joint_times[time_i + 1]
                if (not (joint_time_0 < time_diff < joint_time_1)):
                    continue

                t = (time_diff - joint_time_0) / (joint_time_1 - joint_time_0)
                target_joints[name] = bezier(time_i, t, joint_keys)
                if (name == 'LHipYawPitch'):
                    target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
