#!/usr/local/bin/python3
# coding=utf-8
import time

import libpyauboi5

RS_SUCC = 0

print(__name__)
print("111")

# pdb.set_trace();

# 获取系统当前时间
def get_local_time():
    return time.strftime("%b %d %Y %H:%M:%S:", time.localtime(time.time()))


def get_local_mes_time():
    ct = time.time()
    data_msecs = (ct-int(ct))*1000
    time_msecs = str("%03d" % data_msecs)
    return time_msecs



# 机械臂事件
def robot_event(event):
    print("catch event{0}".format(event))


def main():
    # 初始化libpyauboi5库
    result = libpyauboi5.initialize()
    print("{0} initialize result={1}".format(get_local_time(), result))

    # 创建robot service控制上下文
    RSHD = libpyauboi5.create_context()
    print("create_context RSHD={0}".format(RSHD))

    # 登陆服务器
    #ip_addr = "127.0.0.1"
    ip_addr = '192.168.1.115'
    port = 8899

    print("{0}{1}-------log start".format(get_local_time(),get_local_mes_time()))
    
    count = 1
    while count < 2:


        print("************************mission start({0})*****************************".format(count))

        result = libpyauboi5.login(RSHD, ip_addr, port)

        if result == RS_SUCC:
        # 登陆成功
            print("login {0}:{1} succ.".format(ip_addr, port))
            print("-----------------------登陆成功----------------------------")
            print("{0}{1}-------log end".format(get_local_time(),get_local_mes_time()))


        # 初始化机械臂控制全局属性

            libpyauboi5.init_global_move_profile(RSHD)


            

            joint_radian_3 = (0.000501, -0.126556, -1.319983, 0.379898, -1.573658, 0.000501)
            result = libpyauboi5.move_joint(RSHD, joint_radian_3)
            print("机械臂轴动 first point")


            #time.sleep(1)
            
            #movegroup_point1_joint = {0.000501, -0.126556, -1.319983, 0.379898, -1.573658, 0.000501};
            #movegroup_point2_joint = {-0.341983, -0.172833, -1.357454, 0.387596, -1.574341, -0.341984};
            #movegroup_point3_joint = {-0.341983, -0.172833, -1.357454, 0.387596, -1.574341, -0.341984};
            #movegroup_point4_joint = {-0.502177, -0.082374, -1.093226, 0.560781, -1.574522, -0.502179};
            #movegroup_point5_joint = {-0.340724, 0.020273, -0.783671, 0.768277, -1.574339, -0.340724};
            #movegroup_point6_joint = {-0.340724, 0.020273, -0.783671, 0.768277, -1.574339, -0.340724};
            #movegroup_point7_joint = {0.001463, 0.070957, -0.730550, 0.771820, -1.573656, 0.001464};
            #movegroup_point8_joint = {0.001463, 0.070957, -0.730550, 0.771820, -1.573656, 0.001464};
            #movegroup_point9_joint = {0.163451, 0.008600, -1.008267, 0.556889, -1.573210, 0.163451};
            #movegroup_point10_joint = {0.000501, -0.126556, -1.319983, 0.379898, -1.573658, 0.000501};

	    
            #movegroup  直线加圆弧不减速40mm/s  200次循环
            # 轨迹类型 多段直线 POLYLINE 枚举：15
            # 轨迹类型 圆弧 ARC 枚举：90    
            libpyauboi5.start_movegroup(RSHD)
		
            

            libpyauboi5.init_global_move_profile(RSHD)
            libpyauboi5.set_end_max_line_velc(RSHD,0.04)
            libpyauboi5.set_end_max_line_acc(RSHD,1.570796)
            libpyauboi5.set_end_max_angle_velc(RSHD,2)
            libpyauboi5.set_end_max_angle_acc(RSHD,13.089969)
            libpyauboi5.set_end_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_angle_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_arrival_ahead_distance(RSHD,0.01)
            libpyauboi5.add_waypoint(RSHD,(0.000501, -0.126556, -1.319983, 0.379898, -1.573658, 0.000501))
            libpyauboi5.add_waypoint(RSHD,(-0.341983, -0.172833, -1.357454, 0.387596, -1.574341, -0.341984))
            libpyauboi5.move_track_unblock(RSHD,15)

            libpyauboi5.init_global_move_profile(RSHD)
            libpyauboi5.set_end_max_line_velc(RSHD,0.04)
            libpyauboi5.set_end_max_line_acc(RSHD,1.570796)
            libpyauboi5.set_end_max_angle_velc(RSHD,2)
            libpyauboi5.set_end_max_angle_acc(RSHD,13.089969)
            libpyauboi5.set_end_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_angle_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_arrival_ahead_distance(RSHD,0.01)
            libpyauboi5.add_waypoint(RSHD,(-0.341983, -0.172833, -1.357454, 0.387596, -1.574341, -0.341984))
            libpyauboi5.add_waypoint(RSHD,(-0.502177, -0.082374, -1.093226, 0.560781, -1.574522, -0.502179))
            libpyauboi5.add_waypoint(RSHD,(-0.340724, 0.020273, -0.783671, 0.768277, -1.574339, -0.340724))
            libpyauboi5.move_track_unblock(RSHD,90)


            libpyauboi5.init_global_move_profile(RSHD)
            libpyauboi5.set_end_max_line_velc(RSHD,0.04)
            libpyauboi5.set_end_max_line_acc(RSHD,1.570796)
            libpyauboi5.set_end_max_angle_velc(RSHD,2)
            libpyauboi5.set_end_max_angle_acc(RSHD,13.089969)
            libpyauboi5.set_end_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_angle_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_arrival_ahead_distance(RSHD,0.01)
            libpyauboi5.add_waypoint(RSHD,(-0.340724, 0.020273, -0.783671, 0.768277, -1.574339, -0.340724))
            libpyauboi5.add_waypoint(RSHD,(0.001463, 0.070957, -0.730550, 0.771820, -1.573656, 0.001464))
            libpyauboi5.move_track_unblock(RSHD,15)



            libpyauboi5.init_global_move_profile(RSHD)
            libpyauboi5.set_end_max_line_velc(RSHD,0.04)
            libpyauboi5.set_end_max_line_acc(RSHD,1.570796)
            libpyauboi5.set_end_max_angle_velc(RSHD,2)
            libpyauboi5.set_end_max_angle_acc(RSHD,13.089969)
            libpyauboi5.set_end_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_angle_max_jerk_acc_ratio(RSHD,0.25)
            libpyauboi5.set_arrival_ahead_distance(RSHD,0.01)
            libpyauboi5.add_waypoint(RSHD,(0.001463, 0.070957, -0.730550, 0.771820, -1.573656, 0.001464))
            libpyauboi5.add_waypoint(RSHD,(0.163451, 0.008600, -1.008267, 0.556889, -1.573210, 0.163451))
            libpyauboi5.add_waypoint(RSHD,(0.000501, -0.126556, -1.319983, 0.379898, -1.573658, 0.000501))
            libpyauboi5.move_track_unblock(RSHD,90)



                
            libpyauboi5.endof_movegroup(RSHD)
            libpyauboi5.wait_movegroup_finish(RSHD)
                


        
            count += 1
            print("-----------------------退出登陆----------------------------")
            print("-----------------------执行次数={0}----------------------------".format(count))
        # 注销登陆
            libpyauboi5.logout(RSHD)
            print("logout.")

    else:  # 登陆失败
        print("login {0}:{1} failed.".format(ip_addr, port))  # 释放robot service控制上下文

    # 删除上下文
    libpyauboi5.destory_context(RSHD)

    # 反初始化libpyauboi5库
    libpyauboi5.uninitialize()


if __name__ == "__main__":

    
    
        main()
        
        print("************************mission completed!************************")
