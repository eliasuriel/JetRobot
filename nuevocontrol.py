import numpy as np
import matplotlib.pyplot as plt


class Control:
    def init(self) -> None:
        pass

    def fun_control(self, x1, y1, theta1, xdLider, ydLider, thetadLider):
        r = 0.048
        lx = 0.10
        ly = 0.11
        dt = 0.01 #CHECARLO EN LA SIGUIENTE ITERACION

        # Definir el punto deseado al que el agente debe ir
        # xdDeseado = 1
        # ydDeseado = 0.5
        # thetad = np.pi/2

        x2 = -1
        y2 = 0
        theta2 = np.pi / 2


        xLider = 0
        yLider = 0
        thetaLider = np.pi / 2

        kpr = 10
        k = 10
        vmax = 0.5
        wmax = 0.5
        d_d = 0.01
        d_dL = 0
        dLider = 0

        #LIDER 
        dis_Lider_pd =np.sqrt(pow((xLider - xdLider),2)) + np.sqrt(pow((yLider - ydLider),2))
        Mag_vel_Lider = k * (dis_Lider_pd - d_dL)

        pos_Lider =  np.array([xLider, yLider])
        Dir_vel_Lider = (pos_Lider - [xdLider, ydLider]) / dis_Lider_pd

        Vel_I_Lider = Mag_vel_Lider * Dir_vel_Lider
        Rot_Lider = np.array([[-np.cos(thetaLider), -np.sin(thetaLider)],
                            [np.sin(thetaLider), -np.cos(thetaLider)]])
        Vec_Lider = np.dot(Rot_Lider, Vel_I_Lider)

        #Saturación Lider Lineal
        if np.linalg.norm(Vec_Lider) > vmax:
            Vec_Lider = Vec_Lider * vmax / np.linalg.norm(Vec_Lider)
        Vx_Lider = Vec_Lider[0]
        Vy_Lider = Vec_Lider[1]
        
        # Saturación LIDER Rotacional 
        wLider = -kpr * (thetaLider - thetadLider)
        if wLider > wmax:
            wLider = wmax
        if wLider < -wmax:
            wLider = -wmax

        # Simulación de la velocidad del lider
        xp_Lider = Vx_Lider * np.cos(thetaLider) - Vy_Lider * np.sin(thetaLider)
        yp_Lider = Vx_Lider * np.sin(thetaLider) + Vy_Lider * np.cos(thetaLider)
        thetap_Lider = wLider 

        #Integral del lider
        xLider = xLider + xp_Lider * dt
        yLider = yLider + yp_Lider * dt
        thetaLider = thetaLider + thetap_Lider * dt

        #Saturación del angulo 
        if thetaLider > np.pi:
            thetaLider = thetaLider - 2 * np.pi
        if thetaLider < -np.pi:
            thetaLider = thetaLider + 2 * np.pi

        # Distancia entre los dos agentes
        dis_agentes = np.sqrt(pow((x1 - x2),2) + pow((y1 - y2),2))


        dLider = 0.1        #distancia de los agentes con el lider
        xd1 = xLider + dLider * np.cos((np.pi/2) - thetaLider)      #originalmente cos(thetalider)
        yd1 = yLider - dLider * np.sin((np.pi/2) - thetaLider)

        xd2 = xLider - dLider * np.cos(np.pi/2 - thetaLider)
        yd2 = yLider + dLider * np.sin(np.pi/2 - thetaLider)

        d1Agente = np.sqrt(pow((x1 - xd1),2) + pow((y1 - yd1),2))
        d2Agente = np.sqrt(pow((x2 - xd2),2) + pow((y2 - yd2),2))

        Mag_vel_ag = k * (dis_agentes - d_d)
        Mag_vel_age1 = k * (d1Agente - dLider)
        Mag_vel_age2 = k * (d2Agente - dLider)

        p1 = np.array([x1, y1])
        p2 = np.array([x2, y2])
        pLider = np.array([xLider, yLider])

        Dir_vel_12 = (p1 - p2) / dis_agentes
        Dir_vel_1Ag = (p1 - pLider) / d1Agente

        Dir_vel_21 = (p2 - p1) / dis_agentes
        Dir_vel_2Ag = (p2 - pLider) / d2Agente

        Vel_I_1 = Mag_vel_ag * Dir_vel_12 + Mag_vel_age1 * Dir_vel_1Ag
        Rot_1 = np.array([[-np.cos(theta1), -np.sin(theta1)],
                        [np.sin(theta1), -np.cos(theta1)]])
        Vec_1 = np.dot(Rot_1, Vel_I_1)

        Vel_I_2 = Mag_vel_ag * Dir_vel_21 + Mag_vel_age2 * Dir_vel_2Ag
        Rot_2 = np.array([[-np.cos(theta2), -np.sin(theta2)],
                        [np.sin(theta2), -np.cos(theta2)]])
        Vec_2 = np.dot(Rot_2, Vel_I_2)

        if np.linalg.norm(Vec_1) > vmax:
            Vec_1 = Vec_1 * vmax / np.linalg.norm(Vec_1)
        Vx_1 = Vec_1[0]
        Vy_1 = Vec_1[1]

        # errorx = abs(xd1-x1)
        # errory = abs(yd1-y1)

        theta_e = (theta1 - thetaLider)
        # if (theta_e < np.pi/8 and theta_e > -np.pi/8):
        #     theta_e = 0

        w1 = -kpr * (theta_e)
        if w1 > wmax:
            w1 = wmax
        if w1 < -wmax:
            w1 = -wmax

        rob1_w1 = (-(1/r) * (Vy_1 - Vx_1 - (lx + ly) * w1))
        rob1_w2 = (-(1/r) * (Vy_1 + Vx_1 + (lx + ly) * w1)) 
        rob1_w3 = (-(1/r) * (Vy_1 + Vx_1 - (lx + ly) * w1))
        rob1_w4 = (-(1/r) * (Vy_1 - Vx_1 + (lx + ly) * w1)) 
        # if ( errorx < 0.30 and errory < 0.30):
        #     rob1_w1 = 0
        #     rob1_w2 = 0
        #     rob1_w3 = 0
        #     rob1_w4 = 0
        if np.linalg.norm(Vec_2) > vmax:
            Vec_2 = Vec_2 * vmax / np.linalg.norm(Vec_2)
        Vx_2 = Vec_2[0]
        Vy_2 = Vec_2[1]

        w2 = -kpr * (theta2 - thetaLider)
        if w2 > wmax:
            w2 = wmax
        if w2 < -wmax:
            w2 = -wmax

        # rob2_w1 = (1/r) * (Vx_2 - Vy_2 - (lx + ly) * w1)
        # rob2_w2 = (1/r) * (Vx_2 + Vy_2 + (lx + ly) * w1)
        # rob2_w3 = (1/r) * (Vx_2 + Vy_2 - (lx + ly) * w1)
        # rob2_w4 = (1/r) * (Vx_2 - Vy_2 + (lx + ly) * w1)

        xp_2 = Vx_2 * np.cos(theta2) - Vy_2 * np.sin(theta2)
        yp_2 = Vx_2 * np.sin(theta2) + Vy_2 * np.cos(theta2)
        thetap_2 = w2


        x2 = x2 + xp_2 * dt
        y2 = y2 + yp_2 * dt
        theta2 = theta2 + thetap_2 * dt
        if theta2 > np.pi:
            theta2 = theta2 - 2 * np.pi
        if theta2 < -np.pi:
            theta2 = theta2 + 2 * np.pi

        return rob1_w1, rob1_w2, rob1_w3, rob1_w4