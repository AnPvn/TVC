import math

q = [1, 0, 0, 0]
angulos_de_euler = [0, 0, 0]
dt = 1/1000

if __name__ == '__main__':
    while(True):
        gx = float(input('gx: '))*(-1)
        gy = float(input('gy: '))*(-1)
        gz = float(input('gz: '))*(-1)
        q[0] += (0.5)*(q[1]*gx - q[2]*gy - q[3]*gz)*dt;
        q[1] += (0.5)*(q[0]*gx + q[2]*gz - q[3]*gy)*dt;
        q[2] += (0.5)*(q[0]*gy - q[1]*gz + q[3]*gx)*dt;
        q[3] += (0.5)*(q[0]*gz + q[1]*gy - q[2]*gx)*dt;
        angulos_de_euler[0] = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1-2*(q[1]*q[1] + q[2]*q[2]));
        angulos_de_euler[1] = math.asin(2*(q[0]*q[2] - q[3]*q[1]));
        angulos_de_euler[2] = math.atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
        for a in angulos_de_euler:
            print(a*(57296/1000))
    
