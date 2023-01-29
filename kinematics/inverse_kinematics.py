'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    # def from_trans(self, matrix):
    #     theta_x, theta_y, theta_z = 0, 0, 0
    #     x, y, z = matrix[3, 0], matrix[3, 1], matrix[3, 2]
    #     # if T[0, 0] == 1:
    #     #     theta_x = atan2(T[2, 1], T[1, 1])
    #     # elif T[1, 1] == 1:
    #     #     theta_y = atan2(T[0, 2], T[0, 0])
    #     # elif T[2, 2] == 1:
    #     #     theta_z = atan2(T[1, 0], T[0, 0])
    #     return np.array([x, y, z, theta_x, theta_y, theta_z])
    
    def from_trans(self, matrix):
        x, y, z = matrix[3, 0], matrix[3, 1], matrix[3, 2]
        return np.array([x, y, z])
 

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        # YOUR CODE HERE
        lambda_ = 0.1
        max_step = 0.1
        all_joints = (self.perception.joint).copy()
        target = (self.from_trans(transform)).T

        for i in range(1000):
            self.forward_kinematics(all_joints)
            Ts = self.get_Ts(effector_name)
            Te = np.array([self.from_trans(Ts[-1])]).T
            
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            
            T = np.array([self.from_trans(i) for i in Ts[:]]).T
            J = Te - T
            J[-1, :] = 1
            JTJJT = J.T @ (np.linalg.pinv(J @ J.T))
            d_theta = lambda_ * (JTJJT @ e)
            for i, name in enumerate(self.chains[effector_name]):
                all_joints[name] += np.asarray(d_theta.T)[0, i]
            if np.linalg.norm(d_theta) < 1e-4:
                break
        return all_joints

    def get_Ts(self, effector_name):
        T = [0] * len(self.chains[effector_name])
        for i, name in enumerate(self.chains[effector_name]):
            T[i] = self.transforms[name] 
        return T

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE        
        angles = self.inverse_kinematics(effector_name,transform)
        names = []
        times = []
        keys = []
        for name in self.chains[effector_name]:
            names.append(name)
            times.append([2, 5])
            keys.append([[self.perception.joint[name], [3, 0, 0], [3, 0, 0]], [angles[name], [3, 0, 0], [3, 0, 0]]])
        self.keyframes = (names, times, keys)
        print(angles)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 0] = 0.0      # x
    T[-1, 1] = 0.05     # y
    T[-1, 2] = -0.26    # z
    agent.set_transforms('LLeg', T)
    agent.run()
