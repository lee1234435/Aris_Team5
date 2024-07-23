def dh_transform_trash(self, theta, d, a, alpha):
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        
    def forward_kinematics_trash(self, dh_params, thetas):
        T = np.eye(4)
        for i, params in enumerate(dh_params[:-1]):
            theta = params[0] + thetas[i]
            d = params[1]
            a = params[2]
            alpha = params[3]
            T_i = self.dh_transform_trash(theta, d, a, alpha)
            T = np.dot(T, T_i)
        return T

    
    def inverse_kinematics_trash(self, target_pos, target_orientation, dh_params, initial_guess):
        def objective(thetas):
            T = self.forward_kinematics_trash(dh_params, thetas)
            pos = T[:3, 3]
            ori = T[:3, :3]
            pos_error = np.linalg.norm(pos - target_pos)
            ori_error = np.linalg.norm(ori - target_orientation)

            

            return pos_error + ori_error 
        bounds = [(-360, 360), (-150, 150), (-3.5, 300), (-360, 360), (-124, 124), (-360, 360)]
        result = minimize(objective, initial_guess, bounds=bounds)
        return result.x
    
    def process_input_trash(self, x_input, y_input, z_input):
        
        target_pos = np.array([x_input, y_input, z_input])
        target_orientation = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]) 
        initial_guesses = [
            self.current_state,
            np.random.uniform(-180, 180, 6),
            np.random.uniform(-180, 180, 6),
            np.random.uniform(-180, 180, 6),
        ]
        solutions_trash = []
        for initial_guess in initial_guesses:
            thetas_trash = self.inverse_kinematics_trash(target_pos, target_orientation, self.dh_params, initial_guess)
            solutions_trash.append(thetas_trash)
        min_move_solution_trash = min(solutions_trash, key=lambda thetas_trash: np.linalg.norm(thetas_trash - self.current_state))
        self.current_state = min_move_solution_trash
        thetas_deg_trash = self.current_state
        self.solutions_trash.append(thetas_deg_trash)
        
        print(f"Move joint angles (in degrees): theta1={thetas_deg_trash[0]:.2f}, theta2={thetas_deg_trash[1]:.2f}, theta3={thetas_deg_trash[2]:.2f}, theta4={thetas_deg_trash[3]:.2f}, theta5={thetas_deg_trash[4]:.2f}, theta6={thetas_deg_trash[5]:.2f}")
        code = self._arm.set_servo_angle(angle=thetas_deg_trash, speed=20, mvacc=500, wait=False)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        
        code = self._arm.set_servo_angle(angle=[thetas_deg_trash[0],thetas_deg_trash[1],thetas_deg_trash[2],thetas_deg_trash[3],thetas_deg_trash[4]+self.offset_deg], speed=20, mvacc=500, wait=False)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)



    def motion_home_trash1(self):    ###### 1사분면에 쓰레기가 있는 경우 #########   
        try:
            print("1")
            code = self._arm.set_position(*[-104.5, -115.0, 313.6, -75.8, 85.7, 153.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-72.4, -197.3, 347.0, -38.6, 81.1, -151.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[129.2, -215.2, 331.9, -60.7, 78.1, -119.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[216.6, -127.4, 327.7, -73.2, 83.6, -103.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))

        

    def motion_home_trash3(self):      
        code = self._arm.set_position(*[-163.6, 5.4, 288.4, -17.0, 87.7, 159.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-94.1, 133.3, 287.8, 42.6, 87.0, 164.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[72.2, 143.7, 282.5, 75.3, 87.1, 136.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[158.3, 21.6, 280.3, -78.2, 87.8, -73.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)

    



    def tracking_a_banana(self):
        code = self._arm.set_position(*[-180.5, -75.9, 275.7, -78.9, 79.9, 127.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-171.6, -95.5, 217.8, -166.9, 80.3, 52.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-191.0, -113.9, 208.2, 175.1, 79.2, 42.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-227.0, -135.6, 199.6, 175.7, 79.5, 48.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(4)
        code = self._arm.set_position(*[-214.0, -132.8, 344.0, -61.5, 82.5, 168.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-14.9, -218.3, 333.5, 57.4, 83.0, -36.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        self.tracking_home()

    def tracking_b_choco(self):
        code = self._arm.set_position(*[-164.7, -48.6, 213.8, 178.2, 76.6, 59.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-136.8, -57.5, 198.0, 124.1, 83.6, 27.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-133.5, -99.0, 195.3, 176.2, 83.8, 77.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-129.4, -122.4, 194.4, -176.5, 83.0, 78.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-130.0, -122.7, 202.3, 75.4, 88.4, -29.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(4)
        code = self._arm.set_position(*[-155.8, -133.7, 284.0, -56.4, 85.3, -165.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-67.9, -199.1, 329.8, 32.4, 78.3, -66.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-9.4, -196.3, 344.8, 78.7, 84.7, -15.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        self.tracking_home()

    def tracking_c_strawberry(self):
        code = self._arm.set_position(*[-141.4, 10.7, 212.0, -150.4, 81.8, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-139.7, 8.7, 211.9, -161.9, 82.4, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-123.5, -30.0, 211.0, -171.7, 81.9, 79.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-75.2, -92.1, 210.7, 167.9, 81.5, 102.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-74.7, -96.2, 206.9, 176.3, 76.3, 110.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-54.3, -131.9, 208.3, 176.4, 77.7, 107.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-54.7, -133.5, 221.1, 95.0, 87.4, 25.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(4)
        code = self._arm.set_position(*[-63.2, -137.3, 276.3, -14.4, 84.7, -94.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-17.4, -176.7, 313.4, 177.9, 89.2, 85.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        self.tracking_home()

    def trash(self,input_x,input_y):

        try:
            ################ home 근처 쓰레기 ###############################
            if input_x < -110 :
                print("Home위치 쓰레기 감지")
                self.motion_home()
                self.current_state=np.array([179.2, -42.1, 7.4, 186.7, 41.5, -1.6])
                if 0< input_y <50  :
                    self.offset_x=15                            ###self.offset_x,y 실험값
                    self.offset_y=-15
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90                   ###self.offset_deg 실험값
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif 50 <= input_y <= 150 :
                    self.offset_x=23
                    self.offset_y=-15
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif -13 <= input_y <=-0  :
                    self.offset_x=12.5
                    self.offset_y=-5
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif  -20 <= input_y <-13  :
                    self.offset_x=0
                    self.offset_y=0
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif  -50 <= input_y <-20  :
                    self.offset_x= 10
                    self.offset_y=-20
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif  input_y <-50  :   #####################okay#####################3
                    self.offset_x= 5
                    self.offset_y=-10
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
               
               
                time.sleep(10)
                code = self._arm.close_lite6_gripper()
                if not self._check_code(code, 'close_lite6_gripper'):
                    return
                time.sleep(5)
                self.motion_home()
                time.sleep(1)
                self.throw='Home'
                self.throw_trash()

            elif -110 <= input_x <=160 :
                print("That position is out of Boundary")

            
            elif 160 < input_x < 380:
                
                if input_y <= 0 :
                    ################ 1사분면 근처 쓰레기 ############################
                    print("1사분면 쓰레기 감지")
                    self.motion_home_trash1()
                    self.current_state = np.array([329.5,-7,33.6,179.6,51.2,-5.8])

                    if 160 < input_x < 270 :
                        self.offset_deg=-94
                        if input_x <225 :
                            self.offset_x= -20
                            self.offset_y=-10
                        elif 225<= input_x :
                            self.offset_x= -27.5
                            self.offset_y=-10

                        self.process_input_trash(input_x+self.offset_x,input_y+self.offset_y,332)

                    elif 270 <= input_x <380 :
                        self.offset_deg= -95
                        if input_x <300 :
                            self.offset_x= -27.5
                            self.offset_y=-10

                        elif 300< input_x <350 :
                            self.offset_x= -31
                            self.offset_y=-10

                        elif 350<= input_x :
                            self.offset_x= -35
                            self.offset_y=-10
                        self.process_input_trash(input_x+self.offset_x,input_y+self.offset_y,332)
                        

                    time.sleep(10)
                    code = self._arm.close_lite6_gripper()
                    if not self._check_code(code, 'close_lite6_gripper'):
                        return
                    time.sleep(4)
                    self.process_input(216.6,-127.4,327.7)
                    self.throw='trash1'
                    time.sleep(1)
                    self.throw_trash()

                elif input_y >0 :
                    ################ 3사분면 근처 쓰레기 ############################
                    print("3사분면 쓰레기 감지")
                    self.motion_home_trash3()
                    self.current_state =np.array([9.4, -31.9, 4, 185.1, 54.7, -5.1])
                    

                    if 160 < input_x < 270 :
                        self.offset_deg=-96
                        self.process_input_trash(input_x,input_y-3,332)

                    elif 270 <= input_x <380 :
                        self.offset_deg= -95
                        self.process_input_trash(input_x,input_y-3,332)

                    time.sleep(20)

                    code = self._arm.close_lite6_gripper()
                    if not self._check_code(code, 'close_lite6_gripper'):
                        return
                    time.sleep(4)
                    

                    
                    self.process_input(158.3,-21.6,280.3)
            
        except:
            self.motion_home()
            print("That position is out of Boundary")


    def throw_trash(self):
            if self.throw == 'Home':

               

                code = self._arm.set_position(*[-154.8, -7.0, 320.0, -37.6, 81.6, 142.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-124.0, 75.9, 320.4, -179.4, 86.5, -35.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-134.1, 100.9, 285.9, 107.6, 80.4, -119.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(3)
                code = self._arm.open_lite6_gripper()
                if not self._check_code(code, 'open_lite6_gripper'):
                    return
                time.sleep(3)
                code = self._arm.stop_lite6_gripper()
                if not self._check_code(code, 'stop_lite6_gripper'):
                    return
                time.sleep(1)
            
                code = self._arm.set_position(*[-159.6, 31.7, 294.3, 166.7, 83.5, -27.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)


            elif self.throw == 'trash1' :
                code = self._arm.set_position(*[216.6, -127.4, 327.7, -58.3, 83.4, -89.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[4.2, -244.0, 352.2, -26.2, 59.2, -112.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-231.5, -53.0, 362.1, -1.0, 51.5, -176.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-208.6, 1.8, 339.9, 51.2, 83.0, -141.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-176.8, 62.7, 321.8, 116.2, 80.8, -93.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-128.4, 114.8, 279.0, 157.0, 63.7, -77.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(3)
                code = self._arm.open_lite6_gripper()
                if not self._check_code(code, 'open_lite6_gripper'):
                    return
                time.sleep(5)
                code = self._arm.stop_lite6_gripper()
                if not self._check_code(code, 'stop_lite6_gripper'):
                    return  
                time.sleep(1)
                code = self._arm.set_position(*[-174.7, -0.9, 311.2, -91.2, 86.0, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
            else:
                print("Not designed")
