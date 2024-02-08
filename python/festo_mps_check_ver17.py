import open3d as o3d
import numpy as np

import time

import math
#写真取る用
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

def draw_pc(pt,cl):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pt)
    pcd.colors = o3d.utility.Vector3dVector(cl)
    o3d.visualization.draw_geometries([pcd])

#MPSの端をfor文で確認して見つけていく
def clip_one_mater(pt,cl,mode):
    pt_c = []
    cl_c = []
    mps_xmin = 0
    mps_xmax = 0
    mps_zmin = 100
    mps_zmax = 0

    for i in range(len(pt)):
        pt_x = pt[i][0] * 100
        pt_y = pt[i][1] * 100
        pt_z = abs(pt[i][2]) * 100
        if(pt_x >= -50 and pt_x <= 50 and pt_z <= 100 and pt_y >= -31):
            pt_c.append(pt[i])
            cl_c.append(cl[i])
            if(pt_x < mps_xmin and pt_y >= -31 and pt_y <= -30):
                mps_xmin = int(pt_x)
            if(pt_x > mps_xmax and pt_y >= -31 and pt_y <= -30):
                mps_xmax = int(pt_x)
            if(pt_z < mps_zmin and pt_y >= -31 and pt_y <= -30):
                mps_zmin = int(pt_z)
                mps_zmax = mps_zmin + 35
            #if(pt_y >= -21 and pt_y <= -20):
            #    if(pt_x < mps_xmin):
            #        mps_xmin = int(pt_x)
            #    if(pt_x > mps_xmax):
            #        mps_xmax = int(pt_x)
            #    if(pt_z < mps_zmin):
            #        mps_zmin = int(pt_z)
            #        mps_zmax = mps_zmin + 35
    #print(mps_xmin,mps_xmax,mps_zmin,mps_zmax)
    #print(len(pt_c))
    
    #print("pt_x")
    #print(pt_x)
    #print("pt_y")
    #print(pt_y)
    #print("pt_z")
    #print(pt_z)

    mps_xmin -= 1
    mps_xmax += 1
    mps_zmin -= 1
    mps_zmax += 1

    if(mode == 2):
        print(mps_xmin,mps_xmax,mps_zmin,mps_zmax)
        draw_pc(pt_c,cl_c)

    return(pt_c,cl_c,mps_xmin,mps_xmax,mps_zmin,mps_zmax)

def check_module(direction,pc_c,position,pt,cl,mode):
    pt_c = []
    cl_c = []
    count = 0

    ymin = mps_ymin + position[2]
    ymax = mps_ymin + position[3]
    if(direction == 0):
        xmin = mps_xmin + position[0]
        xmax = mps_xmax - position[1]
        zmin = mps_zmin + position[4]
        zmax = mps_zmax - position[5]
    else:
        xmin = mps_xmin + position[1]
        xmax = mps_xmax - position[0]
        zmin = mps_zmin + position[5]
        zmax = mps_zmax - position[4]
    
    #print("xmin,xmax,ymin,ymax,zmin,zmax")
    #print(xmin,xmax,ymin,ymax,zmin,zmax)
    #print("xmin"+str(xmin))
    #print("position[0]"+str(position[0]))
    #print("xmax"+str(xmax))
    #print("position[1]"+str(position[1]))
    #print("ymin"+str(ymin))
    #print("ymax"+str(ymax))
    #print("zmin"+str(zmin))
    #print("position[4]"+str(position[4]))
    #print("zmax"+str(zmax))
    #print("position[5/mo/]"+str(position[5]))

    for i in range(len(pt)):
        pt_x = pt[i][0] * 100
        pt_y = pt[i][1] * 100
        pt_z = abs(pt[i][2]) * 100
        if(pt_x >= xmin and pt_x <= xmax
                and pt_y >= ymin and pt_y <= ymax
                and pt_z >= zmin and pt_z <= zmax):
            count += 1
            if(mode == 2):
                pt_c.append(pt[i])
                cl_c.append(cl[i])

    if(mode == 2):
        draw_pc(pt_c,cl_c)

    up = pc_c * (1 + alpha)
    down = pc_c * (1 - alpha)
    #print("count")
    #print(count)
    #print("up")
    #print(up)
    #print("down")
    #print(down)
    #print(len(pt))

    if(mode == 0):
        print(count)
        return count
    elif(mode == 2):
        print(xmin,xmax,ymin,ymax,zmin,zmax)
        print(count)
        return 0
    else:
        if(count >= down and count <= up):
            print("OK")
            return 1
        else:
            print("NO")
            return 0

mps_name = ["base",
            "cap",
            "ring",
            "storage",
            "delivery"]
mps_direction = ["in","out"]

x = 0
#Base Station-------------------------------------------------
base_module_name = ["belt","airmeter","displayport1","displayport2","stack_magazine1","stack_magazine2","stack_magazine3"]
base_module_position = [[ 25, 32, 0, 14, 0, 0],
                       [ 62, 0, 0, 15, 23, 3],
                       [ 43, 17, 0, 14, 0, 9],
                       [ 59, 2, 0, 14, 9, 19],
                       [ 0, 30, 8, 32, 0, 19],
                       [ 0, 30, 8, 32, 19, 0],
                       [ 35, 8, 8, 32, 11, 3]]

base_module_pc_c = [[ 2028.5, 1238.4],
                    [ 141.9,  1.6],
                    [ 440.4,  444.3],
                    [ 194.9, 9.7],
                    [ 731.8, 127.1],
                    [ 636.4, 544.8],
                    [ 523.9, 127.1]]
#--------------------------------------------------------------

#Cup station-------------------------------------------------------------------------------
cap_module_name = ["belt","airmeter","displayport1","displayport2","slider","cap","signal"]
cap_module_position = [[ 25, 32, 0, 14, 0, 0],
                             [ 0, 63, 0, 16, 24, 0],
                             [ 44, 11, 0, 16, 0, 26.5],
                             [ 44, 11, 0, 16, 8.5, 16],
                             [ 0, 42, 0, 35, 3, 0],
                             [ 36, 0, 0, 19, 13, 0],
                             [ 56, 6, 0, 26, 0, 27],]

cap_module_pc_c = [[1011.3, 1672.0],
                         [ 0.0,  0.0],
                         [ 228.4,  328.3],
                         [ 139.3, 198.7],
                         [ 3476.6, 3045.7],
                         [ 1944.5, 1853.7],
                         [ 59.2, 103.0],]
#-----------------------------------------------------------------------------------------

#Ring Station-----------------------------------------------------------------------------------------------
ring_module_name = ["belt","airmeter","displayport1","displayport2","slider","signal","PAP1","PAP2"]
ring_module_position = [[ 25, 32, 0, 15, 0, 0],
                           [ 0, 7, 0, 15, 25, 0],
                           [ 44, 17, 0, 14, 9, 18.5],
                           [ 44, 17, 0, 14, 16.5, 11],
                           [ 57, 0, 0, 20, 14, 0],
                           [ 56, 6, 0, 26, 0, 27],
                           [ 0, 39, 0, 35, 0, 17],
                           [ 0, 39, 0, 35, 18, 0]]

ring_module_pc_c = [[ 1777.9,  1629.1],
                       [ 1499.7,  1868.3],
                       [ 61.1,  45.2],
                       [ 86.6,  52.5],
                       [ 544.0,  733.6],
                       [ 80.9,  84.4],
                       [ 3376.5,  2456.2],
                       [ 1344.9, 2060.3]]
#----------------------------------------------------------------------------------------------------------

#Storage Station-----------------------------------------------------------------------------------------------
storage_module_name = ["belt","airmeter","signal","strage"]
storage_module_position = [[ 32, 25, 0, 14, 0, 0],
                       [ 63, 0, 0, 16, 12, 12],
                       [ 61, 0, 0, 25, 0, 27],
                       [ 6, 13, 0, 35, 0, 0]]

storage_module_pc_c = [[ 1115.4,  1115.4],
                       [  19.0,   19.0],
                       [  0.0,  0.0],
                       [  6357.3,   6357.3],]
#----------------------------------------------------------------------------------------------------------

#Delivery Station-----------------------------------------------------------------------------------------------
delivery_module_name = ["belt","airmeter","slider1","slider2","slider3","signal","relay","conecter"]
delivery_module_position = [[ 25, 26, 0, 18, 0, 0],
                           [ 62, 0, 0, 15, 0, 25],
                           [ 40, 6, 0, 12, 10, 16],
                           [ 40, 6, 0, 12, 19, 8],
                           [ 40, 6, 0, 12, 27, 0],
                           [ 56, 6, 0, 26, 0, 28],
                           [ 7, 54, 0, 9, 23, 0],
                           [ 7, 54, 0, 10, 6, 14]]

delivery_module_pc_c = [[ 1563.9,  1919.5],
                       [ 0.0,  0.0],
                       [ 36.3, 40],
                       [ 432.0,  524.3],
                       [ 0.0,  0.0],
                       [ 0.0,  0.0],
                       [ 0.0, 0.0],
                       [ 0.0, 0.0]]
#----------------------------------------------------------------------------------------------------------

#写真データ取得プログラム------------------------------------------------------------------------------------
def initCamera():
    global align, config, pipeline, profile
    
    align = rs.align(rs.stream.color)
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
def initCamera():
    global align, config, pipeline, profile

    align = rs.align(rs.stream.color)
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    pipeline = rs.pipeline()
    pipeline = rs.pipeline()
    profile = pipeline.start(config)

def rs_filter():
    global decimate, spatial, hole_filling, depth_to_disparity, disparity_to_depth
    # decimarion_filter
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 1)
    # spatial_filter
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 1)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    # hole_filling_filter
    hole_filling = rs.hole_filling_filter()
    # disparity
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

def getDepthdata():
    global align, config, pipeline, profile, decimate, spatial, hole_filling, depth_to_disparity, disparity_to_depth
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    align_to = rs.stream.color
    align = rs.align(align_to)

    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

    num = 0

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        depth_frame = aligned_frames.get_depth_frame()
        # filterをかける
        filter_frame = decimate.process(depth_frame)
        filter_frame = depth_to_disparity.process(filter_frame)
        filter_frame = spatial.process(filter_frame)
        filter_frame = disparity_to_depth.process(filter_frame)
        filter_frame = hole_filling.process(filter_frame)
        depth_frame = filter_frame.as_depth_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        #depth_image = np.asanyarray(filter_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        #cv2.namedWindow('color & depth image', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('color image', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
        #images = cv2.resize(images, dsize=None, fx=0.5,fy=0.5)
        cv2.imshow('color image', cv2.cvtColor(images, cv2.COLOR_RGB2BGR))

        depth = o3d.geometry.Image((depth_image < clipping_distance) * depth_image)
        color = o3d.geometry.Image(color_image)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False);
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
        # Estimate Normal
        pcd.estimate_normals()

        key = cv2.waitKey(1)
        num =1
        print("save {0}".format(num))
        o3d.io.write_point_cloud('./Tply{0}.ply'.format(num), pcd)

        if num == 1:
            print('finish')
            cv2.destroyAllWindows()
            break

    #depth = o3d.geometry.Image((depth_image < clipping_distance) * depth_image)

    #color = o3d.geometry.Image(color_image)

    #rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False);
    #pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    #pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

    pipeline.stop()
    #o3d.io.write_point_cloud('./pc_color.ply', pcd)
    #o3d.visualization.draw_geometries([pcd])

    #写真取得用プログラム終了--------------------------------------------------------------------------------------


alpha = 0.1
mps_ymin = -19.5

IN = 0
OUT = 1

d = -14.4 #角度

if __name__ == "__main__":
    time_pg_sta = time.perf_counter()
    #average(1-10):0
    #resalt(11-40):1
    #one_view:2
    initCamera() #photo
    rs_filter() #photo
    getDepthdata() #photo
    mode = 1

    if(mode == 0):
        #Base Station-------------------------------------------------
        base_belt_in_ave = 0
        base_belt_out_ave = 0
        base_airmeter_in_ave = 0
        base_airmeter_out_ave = 0
        base_displayport1_in_ave = 0
        base_displayport1_out_ave = 0
        base_displayport2_in_ave = 0
        base_displayport2_out_ave = 0
        base_stack_magazine1_in_ave = 0
        base_stack_magazine1_out_ave = 0
        base_stack_magazine2_in_ave = 0
        base_stack_magazine2_out_ave = 0
        base_stack_magazine3_in_ave = 0
        base_stack_magazine3_out_ave = 0
        
        #Cap station-------------------------------------------------------------------------------
        cap_belt_in_ave = 0
        cap_belt_out_ave = 0
        cap_airmeter_in_ave = 0
        cap_airmeter_out_ave = 0
        cap_displayport1_in_ave  = 0
        cap_displayport1_out_ave = 0
        cap_displayport2_in_ave  = 0
        cap_displayport2_out_ave = 0
        cap_slider_in_ave = 0
        cap_slider_out_ave = 0
        cap_cap_in_ave = 0
        cap_cap_out_ave = 0
        cap_signal_in_ave = 0
        cap_signal_out_ave = 0

        #ring station----------------------------------------------------------------------------
        ring_belt_in_ave = 0
        ring_belt_out_ave = 0
        ring_airmeter_in_ave = 0
        ring_airmeter_out_ave = 0
        ring_displayport1_in_ave = 0
        ring_displayport1_out_ave = 0
        ring_displayport2_in_ave = 0
        ring_displayport2_out_ave = 0
        ring_slider_in_ave = 0
        ring_slider_out_ave = 0
        ring_signal_in_ave = 0
        ring_signal_out_ave = 0
        ring_PAP1_in_ave = 0
        ring_PAP1_out_ave = 0
        ring_PAP2_in_ave = 0
        ring_PAP2_out_ave = 0

        #Storage Station-----------------------------------------------------------------------------------------------
        storage_belt_in_ave = 0
        storage_belt_out_ave = 0
        storage_airmeter_in_ave = 0
        storage_airmeter_out_ave = 0
        storage_signal_in_ave = 0
        storage_signal_out_ave = 0
        storage_strage_in_ave = 0
        storage_strage_out_ave = 0

        #Delivery Station-----------------------------------------------------------------------------------------------
        delivery_belt_in_ave = 0
        delivery_belt_out_ave = 0
        delivery_airmeter_in_ave = 0
        delivery_airmeter_out_ave = 0
        delivery_slider1_in_ave = 0
        delivery_slider1_out_ave = 0
        delivery_slider2_in_ave = 0
        delivery_slider2_out_ave = 0
        delivery_slider3_in_ave = 0
        delivery_slider3_out_ave = 0
        delivery_signal_in_ave = 0
        delivery_signal_out_ave = 0
        delivery_relay_in_ave = 0
        delivery_relay_out_ave = 0
        delivery_conecter_in_ave = 0
        delivery_conecter_out_ave = 0

        for i in range(5): #モジュールの数
            for j in range(2): #in/out
                for k in range(1,11): #教師用データの数
                    ptcloud = o3d.io.read_point_cloud("mps_{}_{}/ply{}.ply".format(mps_name[i],mps_direction[j],k))
                    ptcloud = ptcloud.voxel_down_sample(voxel_size=0.005)
                    pt = np.asarray(ptcloud.points)
                    #print("<<pt1>>")
                    #print(pt)
                    y = 0
                    z = 0
                    for n in range(len(pt)):
                        #d = -14.85 #角度
                        d_rad = math.radians(d)
                        y = float(pt[n][1])
                        z = float(pt[n][2])
                        y_rotated = y * math.cos(d_rad) - z * math.sin(d_rad)
                        z_rotated = y * math.sin(d_rad) + z * math.cos(d_rad)
                        pt[n][1] = y_rotated
                        pt[n][2] = z_rotated
                    #print("<<<pt変換後>>>")
                    #print(pt)
                    cl = np.asarray(ptcloud.colors)
                    print("\nmps_{}_{}/ply{}.ply".format(mps_name[i],mps_direction[j],k))

                    pt_c,cl_c,mps_xmin,mps_xmax,mps_zmin,mps_zmax = clip_one_mater(pt,cl,mode)
                    
                    #base
                    if(i == 0):
                        if(j == 0):
                            print("belt")
                            base_belt_in_ave        += check_module(j,base_module_pc_c[0][j],base_module_position[0],pt,cl,mode)
                            print("airmeter")
                            base_airmeter_in_ave    += check_module(j,base_module_pc_c[1][j],base_module_position[1],pt,cl,mode)
                            print("displayport1")
                            base_displayport1_in_ave += check_module(j,base_module_pc_c[2][j],base_module_position[2],pt,cl,mode)
                            print("displayport2")
                            base_displayport2_in_ave += check_module(j,base_module_pc_c[3][j],base_module_position[3],pt,cl,mode)
                            print("stack_magazine1")
                            base_stack_magazine1_in_ave += check_module(j,base_module_pc_c[4][j],base_module_position[4],pt,cl,mode)
                            print("stack_magazine2")
                            base_stack_magazine2_in_ave += check_module(j,base_module_pc_c[5][j],base_module_position[5],pt,cl,mode)
                            print("stack_magazine3")
                            base_stack_magazine3_in_ave += check_module(j,base_module_pc_c[6][j],base_module_position[6],pt,cl,mode)
                        else:
                            print("belt")
                            base_belt_out_ave        += check_module(j,base_module_pc_c[0][j],base_module_position[0],pt,cl,mode)
                            print("airmeter")
                            base_airmeter_out_ave    += check_module(j,base_module_pc_c[1][j],base_module_position[1],pt,cl,mode)
                            print("displayport1")
                            base_displayport1_out_ave += check_module(j,base_module_pc_c[2][j],base_module_position[2],pt,cl,mode)
                            print("displayport2")
                            base_displayport2_out_ave += check_module(j,base_module_pc_c[3][j],base_module_position[3],pt,cl,mode)
                            print("stack_magazine1")
                            base_stack_magazine1_out_ave += check_module(j,base_module_pc_c[4][j],base_module_position[4],pt,cl,mode)
                            print("stack_magazine2")
                            base_stack_magazine2_out_ave += check_module(j,base_module_pc_c[5][j],base_module_position[5],pt,cl,mode)
                            print("stack_magazine3")
                            base_stack_magazine3_out_ave += check_module(j,base_module_pc_c[4][j],base_module_position[4],pt,cl,mode)
                    #cap
                    elif(i == 1):
                        if(j == 0):
                            print("belt")
                            cap_belt_in_ave        += check_module(j,cap_module_pc_c[0][j],cap_module_position[0],pt,cl,mode)
                            print("airmeter")
                            cap_airmeter_in_ave    += check_module(j,cap_module_pc_c[1][j],cap_module_position[1],pt,cl,mode)
                            print("displayport1")
                            cap_displayport1_in_ave += check_module(j,cap_module_pc_c[2][j],cap_module_position[2],pt,cl,mode)
                            print("displayport2")
                            cap_displayport2_in_ave += check_module(j,cap_module_pc_c[3][j],cap_module_position[3],pt,cl,mode)
                            print("silder")
                            cap_slider_in_ave         += check_module(j,cap_module_pc_c[4][j],cap_module_position[4],pt,cl,mode)
                            print("cap")
                            cap_cap_in_ave      += check_module(j,cap_module_pc_c[5][j],cap_module_position[5],pt,cl,mode)
                            print("signal")
                            cap_signal_in_ave      += check_module(j,cap_module_pc_c[6][j],cap_module_position[6],pt,cl,mode)
                        else:
                            print("belt")
                            cap_belt_out_ave        += check_module(j,cap_module_pc_c[0][j],cap_module_position[0],pt,cl,mode)
                            print("airmeter")
                            cap_airmeter_out_ave    += check_module(j,cap_module_pc_c[1][j],cap_module_position[1],pt,cl,mode)
                            print("displayport1")
                            cap_displayport1_out_ave += check_module(j,cap_module_pc_c[2][j],cap_module_position[2],pt,cl,mode)
                            print("displayport2")
                            cap_displayport2_out_ave += check_module(j,cap_module_pc_c[3][j],cap_module_position[3],pt,cl,mode)
                            print("silder")
                            cap_slider_out_ave         += check_module(j,cap_module_pc_c[4][j],cap_module_position[4],pt,cl,mode)
                            print("cap")
                            cap_cap_out_ave      += check_module(j,cap_module_pc_c[5][j],cap_module_position[5],pt,cl,mode)
                            print("signal")
                            cap_signal_out_ave      += check_module(j,cap_module_pc_c[6][j],cap_module_position[6],pt,cl,mode)
                    
                    #ring station
                    elif(i == 2):
                        if(j == 0):
                            print("belt")
                            ring_belt_in_ave        += check_module(j,ring_module_pc_c[0][j],ring_module_position[0],pt,cl,mode)
                            print("airmeter")
                            ring_airmeter_in_ave    += check_module(j,ring_module_pc_c[1][j],ring_module_position[1],pt,cl,mode)
                            print("displayport1")
                            ring_displayport1_in_ave += check_module(j,ring_module_pc_c[2][j],ring_module_position[2],pt,cl,mode)
                            print("displayport2")
                            ring_displayport2_in_ave      += check_module(j,ring_module_pc_c[3][j],ring_module_position[3],pt,cl,mode)
                            print("slider")
                            ring_slider_in_ave     += check_module(j,ring_module_pc_c[4][j],ring_module_position[4],pt,cl,mode)
                            print("signal")
                            ring_signal_in_ave     += check_module(j,ring_module_pc_c[5][j],ring_module_position[5],pt,cl,mode)
                            print("PAP1")
                            ring_PAP1_in_ave     += check_module(j,ring_module_pc_c[6][j],ring_module_position[6],pt,cl,mode)
                            print("PAP2")
                            ring_PAP2_in_ave     += check_module(j,ring_module_pc_c[7][j],ring_module_position[7],pt,cl,mode)
                        else:
                            print("belt")
                            ring_belt_out_ave        += check_module(j,ring_module_pc_c[0][j],ring_module_position[0],pt,cl,mode)
                            print("airmeter")
                            ring_airmeter_out_ave    += check_module(j,ring_module_pc_c[1][j],ring_module_position[1],pt,cl,mode)
                            print("displayport1")
                            ring_displayport1_out_ave += check_module(j,ring_module_pc_c[2][j],ring_module_position[2],pt,cl,mode)
                            print("displayport2")
                            ring_displayport2_out_ave      += check_module(j,ring_module_pc_c[3][j],ring_module_position[3],pt,cl,mode)
                            print("slider")
                            ring_slider_out_ave     += check_module(j,ring_module_pc_c[4][j],ring_module_position[4],pt,cl,mode)
                            print("signal")
                            ring_signal_out_ave     += check_module(j,ring_module_pc_c[5][j],ring_module_position[5],pt,cl,mode)
                            print("PAP1")
                            ring_PAP1_out_ave     += check_module(j,ring_module_pc_c[6][j],ring_module_position[6],pt,cl,mode)
                            print("PAP2")
                            ring_PAP2_out_ave     += check_module(j,ring_module_pc_c[7][j],ring_module_position[7],pt,cl,mode)
                    #Storage Station
                    elif(i == 3):
                        if(j == 0):
                            print("belt")
                            storage_belt_in_ave        += check_module(j,storage_module_pc_c[0][j],storage_module_position[0],pt,cl,mode)
                            print("airmeter")
                            storage_airmeter_in_ave    += check_module(j,storage_module_pc_c[1][j],storage_module_position[1],pt,cl,mode)
                            print("signal")
                            storage_signal_in_ave += check_module(j,storage_module_pc_c[2][j],storage_module_position[2],pt,cl,mode)
                            print("strage")
                            storage_strage_in_ave      += check_module(j,storage_module_pc_c[3][j],storage_module_position[3],pt,cl,mode)
                        else:
                            print("belt")
                            storage_belt_out_ave        += check_module(j,storage_module_pc_c[0][j],storage_module_position[0],pt,cl,mode)
                            print("airmeter")
                            storage_airmeter_out_ave    += check_module(j,storage_module_pc_c[1][j],storage_module_position[1],pt,cl,mode)
                            print("signal")
                            storage_signal_out_ave += check_module(j,storage_module_pc_c[2][j],storage_module_position[2],pt,cl,mode)
                            print("strage")
                            storage_strage_out_ave      += check_module(j,storage_module_pc_c[3][j],storage_module_position[3],pt,cl,mode)
                    #Delivery Station
                    elif(i == 4):
                        if(j == 0):
                            print("belt")
                            delivery_belt_in_ave        += check_module(j,delivery_module_pc_c[0][j],delivery_module_position[0],pt,cl,mode)
                            print("airmeter")
                            delivery_airmeter_in_ave    += check_module(j,delivery_module_pc_c[1][j],delivery_module_position[1],pt,cl,mode)
                            print("slider1")
                            delivery_slider1_in_ave += check_module(j,delivery_module_pc_c[2][j],delivery_module_position[2],pt,cl,mode)
                            print("slider2")
                            delivery_slider2_in_ave      += check_module(j,delivery_module_pc_c[3][j],delivery_module_position[3],pt,cl,mode)
                            print("slider3")
                            delivery_slider2_in_ave      += check_module(j,delivery_module_pc_c[4][j],delivery_module_position[4],pt,cl,mode)
                            print("signal")
                            delivery_signal_in_ave      += check_module(j,delivery_module_pc_c[5][j],delivery_module_position[5],pt,cl,mode)
                            print("select")
                            delivery_relay_in_ave      += check_module(j,delivery_module_pc_c[6][j],delivery_module_position[6],pt,cl,mode)
                            print("select")
                            delivery_relay_in_ave      += check_module(j,delivery_module_pc_c[7][j],delivery_module_position[7],pt,cl,mode)
                        else:
                            print("belt")
                            delivery_belt_out_ave        += check_module(j,delivery_module_pc_c[0][j],delivery_module_position[0],pt,cl,mode)
                            print("airmeter")
                            delivery_airmeter_out_ave    += check_module(j,delivery_module_pc_c[1][j],delivery_module_position[1],pt,cl,mode)
                            print("slider1")
                            delivery_slider1_out_ave += check_module(j,delivery_module_pc_c[2][j],delivery_module_position[2],pt,cl,mode)
                            print("slider2")
                            delivery_slider2_out_ave      += check_module(j,delivery_module_pc_c[3][j],delivery_module_position[3],pt,cl,mode)
                            print("slider3")
                            delivery_slider2_out_ave      += check_module(j,delivery_module_pc_c[4][j],delivery_module_position[4],pt,cl,mode)
                            print("signal")
                            delivery_signal_out_ave      += check_module(j,delivery_module_pc_c[5][j],delivery_module_position[5],pt,cl,mode)
                            print("select")
                            delivery_relay_out_ave      += check_module(j,delivery_module_pc_c[6][j],delivery_module_position[6],pt,cl,mode)
                            print("select")
                            delivery_relay_out_ave      += check_module(j,delivery_module_pc_c[7][j],delivery_module_position[7],pt,cl,mode)


        #Base Station/10-------------------------------------------------
        base_belt_in_ave /= 10
        base_belt_out_ave /= 10
        base_airmeter_in_ave /= 10
        base_airmeter_out_ave /= 10
        base_displayport1_in_ave /= 10
        base_displayport1_out_ave /= 10
        base_displayport2_in_ave /= 10
        base_displayport2_out_ave /= 10
        base_stack_magazine1_in_ave /= 10
        base_stack_magazine1_out_ave /= 10
        base_stack_magazine2_in_ave /= 10
        base_stack_magazine2_out_ave /= 10
        base_stack_magazine3_in_ave /= 10
        base_stack_magazine3_out_ave /= 10

        #Cap station/10-------------------------------------------------------------------------------
        cap_belt_in_ave /= 10
        cap_belt_out_ave /= 10
        cap_airmeter_in_ave /= 10
        cap_airmeter_out_ave /= 10
        cap_displayport1_in_ave  /= 10
        cap_displayport1_out_ave /= 10
        cap_displayport2_in_ave  /= 10
        cap_displayport2_out_ave /= 10
        cap_slider_in_ave /= 10
        cap_slider_out_ave /= 10
        cap_cap_in_ave /= 10
        cap_cap_out_ave /= 10
        cap_signal_in_ave /= 10
        cap_signal_out_ave /= 10

        #ring station/10----------------------------------------------------------------------------
        ring_belt_in_ave /= 10
        ring_belt_out_ave /= 10
        ring_airmeter_in_ave /= 10
        ring_airmeter_out_ave /= 10
        ring_displayport1_in_ave /= 10
        ring_displayport1_out_ave /= 10
        ring_displayport2_in_ave /= 10
        ring_displayport2_out_ave /= 10
        ring_slider_in_ave /= 10
        ring_slider_out_ave /= 10
        ring_signal_in_ave /= 10
        ring_signal_out_ave /= 10
        ring_PAP1_in_ave /= 10
        ring_PAP1_out_ave /= 10
        ring_PAP2_in_ave /= 10
        ring_PAP2_out_ave /= 10

        #Storage Station/10-----------------------------------------------------------------------------------------------
        storage_belt_in_ave /= 10
        storage_belt_out_ave /= 10
        storage_airmeter_in_ave /= 10
        storage_airmeter_out_ave /= 10
        storage_signal_in_ave /= 10
        storage_signal_out_ave /= 10
        storage_strage_in_ave /= 10
        storage_strage_out_ave /= 10

        #Delivery Station/10-----------------------------------------------------------------------------------------------
        delivery_belt_in_ave /= 10
        delivery_belt_out_ave /= 10
        delivery_airmeter_in_ave /= 10
        delivery_airmeter_out_ave /= 10
        delivery_slider1_in_ave /= 10
        delivery_slider1_out_ave /= 10
        delivery_slider2_in_ave /= 10
        delivery_slider2_out_ave /= 10
        delivery_slider3_in_ave /= 10
        delivery_slider3_out_ave /= 10
        delivery_signal_in_ave /= 10
        delivery_signal_out_ave /= 10
        delivery_relay_in_ave /= 10
        delivery_relay_out_ave /= 10
        delivery_conecter_in_ave /= 10
        delivery_conecter_out_ave /= 10


        #Base Station-------------------------------------------------
        print("\n///////////////////")
        print("base")
        print("input")
        print("belt_in{}".format(base_belt_in_ave))
        print("airmeter_in{}".format(base_airmeter_in_ave))
        print("displayport1_in{}".format(base_displayport1_in_ave))
        print("displayport2_in{}".format(base_displayport2_in_ave))
        print("base_stack_magazine1_in{}".format(base_stack_magazine1_in_ave))
        print("base_stack_magazine2_in{}".format(base_stack_magazine2_in_ave))
        print("base_stack_magazine3_in{}".format(base_stack_magazine3_in_ave))
        print("output")
        print("belt_out{}".format(base_belt_out_ave))
        print("airmeter_out{}".format(base_airmeter_out_ave))
        print("displayport1_out{}".format(base_displayport1_out_ave))
        print("displayport2_out{}".format(base_displayport2_out_ave))
        print("base_stack_magazine1_out{}".format(base_stack_magazine1_out_ave))
        print("base_stack_magazine2_out{}".format(base_stack_magazine2_out_ave))
        print("base_stack_magazine3_out{}".format(base_stack_magazine3_out_ave))

        #Cap station-------------------------------------------------------------------------------
        print("\n///////////////////")
        print("cap")
        print("input")
        print("belt_in{}".format(cap_belt_in_ave))
        print("airmeter_in{}".format(cap_airmeter_in_ave))
        print("displayport1_in{}".format(cap_displayport1_in_ave))
        print("displayport2_in{}".format(cap_displayport2_in_ave))
        print("cap_slider_in{}".format(cap_slider_in_ave))
        print("cap_cap_in{}".format(cap_cap_in_ave))
        print("cap_signal_in{}".format(cap_signal_in_ave))
        print("output")
        print("belt_out{}".format(cap_belt_out_ave))
        print("airmeter_out{}".format(cap_airmeter_out_ave))
        print("displayport1_out{}".format(cap_displayport1_out_ave))
        print("displayport2_out{}".format(cap_displayport2_out_ave))
        print("cap_slider_out{}".format(cap_slider_out_ave))
        print("cap_cap_out{}".format(cap_cap_out_ave))
        print("cap_signal_out{}".format(cap_signal_out_ave))

        #ring station/10----------------------------------------------------------------------------
        print("\n///////////////////")
        print("ring")
        print("input")
        print("belt_in{}".format(ring_belt_in_ave))
        print("airmeter_in{}".format(ring_airmeter_in_ave))
        print("ring_displayport1_in{}".format(ring_displayport1_in_ave))
        print("displayport2_in{}".format(ring_displayport2_in_ave))
        print("ring_slider_in_ave{}".format(ring_slider_in_ave))
        print("ring_signal_in{}".format(ring_signal_in_ave))
        print("ring_PAP1_in{}".format(ring_PAP1_in_ave))
        print("ring_PAP2_in{}".format(ring_PAP2_in_ave))
        print("output")
        print("belt_out{}".format(ring_belt_out_ave))
        print("airmeter_out{}".format(ring_airmeter_out_ave))
        print("ring_displayport1_out{}".format(ring_displayport1_out_ave))
        print("displayport2_out{}".format(ring_displayport2_out_ave))
        print("ring_slider_out{}".format(ring_slider_out_ave))
        print("ring_signal_out{}".format(ring_signal_out_ave))
        print("ring_PAP1_out{}".format(ring_PAP1_out_ave))
        print("ring_PAP2_out{}".format(ring_PAP2_out_ave))

        #Storage Station/10-----------------------------------------------------------------------------------------------
        print("\n///////////////////")
        print("storage")
        print("input")
        print("storage_belt_in{}".format(storage_belt_in_ave))
        print("storage_airmeter_in{}".format(storage_airmeter_in_ave))
        print("storage_signal_in{}".format(storage_signal_in_ave))
        print("storage_strage_in{}".format(storage_strage_in_ave))
        print("output")
        print("storage_belt_out{}".format(storage_belt_in_ave))
        print("storage_airmeter_out{}".format(storage_airmeter_in_ave))
        print("storage_signal_out{}".format(storage_signal_in_ave))
        print("storage_strage_out{}".format(storage_strage_in_ave))
        
        #Delivery Station-----------------------------------------------------------------------------------------------
        print("\n///////////////////")
        print("delivery")
        print("input")
        print("delivery_belt_in_ave{}".format(delivery_belt_in_ave))
        print("delivery_airmeter_in{}".format(delivery_airmeter_in_ave))
        print("delivery_slider1_in{}".format(delivery_slider1_in_ave))
        print("delivery_slider2_in{}".format(delivery_slider2_in_ave))
        print("delivery_slider3_in{}".format(delivery_slider3_in_ave))
        print("delivery_signal_out{}".format(delivery_signal_in_ave))
        print("delivery_relay_in{}".format(delivery_relay_in_ave))
        print("delivery_conecter_in{}".format(delivery_conecter_in_ave))
        print("output")
        print("delivery_belt_out_ave{}".format(delivery_belt_out_ave))
        print("delivery_airmeter_out{}".format(delivery_airmeter_out_ave))
        print("delivery_slider1_out{}".format(delivery_slider1_out_ave))
        print("delivery_slider2_out{}".format(delivery_slider2_out_ave))
        print("delivery_slider3_out{}".format(delivery_slider3_out_ave))
        print("delivery_signal_out{}".format(delivery_signal_out_ave))
        print("delivery_relay_out{}".format(delivery_relay_out_ave))
        print("delivery_conecter_out{}".format(delivery_conecter_out_ave))

    #mode == 1
    if(mode == 1):
        for i  in range(5):
            for j in range(2):
                for k in range(11,12):#1つのデータだけでやる
                    #初期値設定
                    base_in = 0
                    base_out = 0
                    cap_in = 0
                    cap_out = 0
                    ring_in = 0
                    ring_out = 0
                    storage_in = 0
                    storage_out = 0
                    storage_in = 0
                    storage_out = 0
                    delivery_in = 0
                    delivery_out = 0

                    #print(i,j,k)
                    #print("mps_{}_{}/ply{}.ply".format(mps_name[i],mps_direction[j],k))
                    time_sta = time.perf_counter()
                    ptcloud = o3d.io.read_point_cloud("/Aply{}.ply".format(k))
                    ptcloud = ptcloud.voxel_down_sample(voxel_size=0.005)
                    pt = np.asarray(ptcloud.points)
                    #角度変換         
                    #print("<<pt1>>")
                    #print(pt)
                    y = 0
                    z = 0
                    for n in range(len(pt)):
                        #d = -14.85 #角度
                        d_rad = math.radians(d)
                        y = float(pt[n][1])
                        z = float(pt[n][2])
                        y_rotated = y * math.cos(d_rad) - z * math.sin(d_rad)
                        z_rotated = y * math.sin(d_rad) + z * math.cos(d_rad)
                        pt[n][1] = y_rotated
                        pt[n][2] = z_rotated
                    #print("<<<pt変換後>>>")
                    #print(pt)

                    cl = np.asarray(ptcloud.colors)
                    #time_end = time.perf_counter()
                    #print("call ply time is {}".format(time_end-time_sta))

                    #time_sta = time.perf_counter()
                    pt_c,cl_c,mps_xmin,mps_xmax,mps_zmin,mps_zmax = clip_one_mater(pt,cl,mode)
                    #time_end = time.perf_counter()
                    #print("clip_one_meter time is {}".format(time_end-time_sta))

                    #time_sta =  time.perf_counter()
                    base_in += check_module(IN,base_module_pc_c[0][IN],base_module_position[0],pt,cl,mode)
                    base_in += check_module(IN,base_module_pc_c[1][IN],base_module_position[1],pt,cl,mode)
                    base_in += check_module(IN,base_module_pc_c[2][IN],base_module_position[2],pt,cl,mode)
                    base_in += check_module(IN,base_module_pc_c[3][IN],base_module_position[3],pt,cl,mode)
                    base_in += check_module(IN,base_module_pc_c[4][IN],base_module_position[4],pt,cl,mode)
                    base_in += check_module(IN,base_module_pc_c[5][IN],base_module_position[5],pt,cl,mode)
                    base_in += check_module(IN,base_module_pc_c[6][IN],base_module_position[6],pt,cl,mode)
                    print("base_in:{}/7\n".format(base_in))

                    base_out += check_module(OUT,base_module_pc_c[0][OUT],base_module_position[0],pt,cl,mode)
                    base_out += check_module(OUT,base_module_pc_c[1][OUT],base_module_position[1],pt,cl,mode)
                    base_out += check_module(OUT,base_module_pc_c[2][OUT],base_module_position[2],pt,cl,mode)
                    base_out += check_module(OUT,base_module_pc_c[3][OUT],base_module_position[3],pt,cl,mode)
                    base_out += check_module(OUT,base_module_pc_c[4][OUT],base_module_position[4],pt,cl,mode)
                    base_out += check_module(OUT,base_module_pc_c[5][OUT],base_module_position[5],pt,cl,mode)
                    base_out += check_module(OUT,base_module_pc_c[6][OUT],base_module_position[6],pt,cl,mode)
                    print("base_out:{}/7\n".format(base_out))
                    
                    cap_in += check_module(IN,cap_module_pc_c[0][IN],cap_module_position[0],pt,cl,mode)
                    cap_in += check_module(IN,cap_module_pc_c[1][IN],cap_module_position[1],pt,cl,mode)
                    cap_in += check_module(IN,cap_module_pc_c[2][IN],cap_module_position[2],pt,cl,mode)
                    cap_in += check_module(IN,cap_module_pc_c[3][IN],cap_module_position[3],pt,cl,mode)
                    cap_in += check_module(IN,cap_module_pc_c[4][IN],cap_module_position[4],pt,cl,mode)
                    cap_in += check_module(IN,cap_module_pc_c[5][IN],cap_module_position[5],pt,cl,mode)
                    cap_in += check_module(IN,cap_module_pc_c[6][IN],cap_module_position[6],pt,cl,mode)
                    print("cap_in:{}/7\n".format(cap_in))
                    
                    cap_out += check_module(OUT,cap_module_pc_c[0][OUT],cap_module_position[0],pt,cl,mode)
                    cap_out += check_module(OUT,cap_module_pc_c[1][OUT],cap_module_position[1],pt,cl,mode)
                    cap_out += check_module(OUT,cap_module_pc_c[2][OUT],cap_module_position[2],pt,cl,mode)
                    cap_out += check_module(OUT,cap_module_pc_c[3][OUT],cap_module_position[3],pt,cl,mode)
                    cap_out += check_module(OUT,cap_module_pc_c[4][OUT],cap_module_position[4],pt,cl,mode)
                    cap_out += check_module(OUT,cap_module_pc_c[5][OUT],cap_module_position[5],pt,cl,mode)
                    cap_out += check_module(OUT,cap_module_pc_c[6][OUT],cap_module_position[6],pt,cl,mode)
                    print("cap_out:{}/7\n".format(cap_out))

                    ring_in += check_module(IN,ring_module_pc_c[0][IN],ring_module_position[0],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[1][IN],ring_module_position[1],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[2][IN],ring_module_position[2],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[3][IN],ring_module_position[3],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[4][IN],ring_module_position[4],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[5][IN],ring_module_position[5],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[6][IN],ring_module_position[6],pt,cl,mode)
                    ring_in += check_module(IN,ring_module_pc_c[7][IN],ring_module_position[7],pt,cl,mode)
                    print("ring_in:{}/8\n".format(ring_in))
                    
                    ring_out += check_module(OUT,ring_module_pc_c[0][OUT],ring_module_position[0],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[1][OUT],ring_module_position[1],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[2][OUT],ring_module_position[2],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[3][OUT],ring_module_position[3],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[4][OUT],ring_module_position[4],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[5][OUT],ring_module_position[5],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[6][OUT],ring_module_position[6],pt,cl,mode)
                    ring_out += check_module(OUT,ring_module_pc_c[7][OUT],ring_module_position[7],pt,cl,mode)
                    print("ring_out:{}/8\n".format(ring_out))


                    storage_in += check_module(IN,storage_module_pc_c[0][IN],storage_module_position[0],pt,cl,mode)
                    storage_in += check_module(IN,storage_module_pc_c[1][IN],storage_module_position[1],pt,cl,mode)
                    storage_in += check_module(IN,storage_module_pc_c[2][IN],storage_module_position[2],pt,cl,mode)
                    storage_in += check_module(IN,storage_module_pc_c[3][IN],storage_module_position[3],pt,cl,mode)
                    print("storage_in:{}/4\n".format(storage_in))

                    storage_out += check_module(OUT,storage_module_pc_c[0][OUT],storage_module_position[0],pt,cl,mode)
                    storage_out += check_module(OUT,storage_module_pc_c[1][OUT],storage_module_position[1],pt,cl,mode)
                    storage_out += check_module(OUT,storage_module_pc_c[2][OUT],storage_module_position[2],pt,cl,mode)
                    storage_out += check_module(OUT,storage_module_pc_c[3][OUT],storage_module_position[3],pt,cl,mode)
                    print("storage_out:{}/4\n".format(storage_out))

                    delivery_in += check_module(IN,delivery_module_pc_c[0][IN],delivery_module_position[0],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[1][IN],delivery_module_position[1],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[2][IN],delivery_module_position[2],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[3][IN],delivery_module_position[3],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[4][IN],delivery_module_position[4],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[5][IN],delivery_module_position[5],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[6][IN],delivery_module_position[6],pt,cl,mode)
                    delivery_in += check_module(IN,delivery_module_pc_c[7][IN],delivery_module_position[7],pt,cl,mode)
                    print("delivery_in:{}/8\n".format(delivery_in))

                    delivery_out += check_module(OUT,delivery_module_pc_c[0][OUT],delivery_module_position[0],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[1][OUT],delivery_module_position[1],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[2][OUT],delivery_module_position[2],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[3][OUT],delivery_module_position[3],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[4][OUT],delivery_module_position[4],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[5][OUT],delivery_module_position[5],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[6][OUT],delivery_module_position[6],pt,cl,mode)
                    delivery_out += check_module(OUT,delivery_module_pc_c[7][OUT],delivery_module_position[7],pt,cl,mode)
                    print("delivery_out:{}/8\n".format(delivery_out))
                    
                    base_in /= 7
                    base_out /= 7
                    cap_in /= 7
                    cap_out /= 7
                    ring_in /= 8
                    ring_out /= 8
                    storage_in /= 4
                    storage_out /= 4
                    delivery_in /= 8
                    delivery_out /= 8

                    def kotae():
                        #割合に変換
                        ans = 0
                        last_check = [base_in,base_out,cap_in,cap_out,ring_in,ring_out,storage_in,storage_out,delivery_in,delivery_out]
                        for w in range(len(last_check)):
                            if (ans < last_check[w]):
                                ans = last_check[w]
                                z = w
                        return z

                    name_check = ["base_in","base_out","cap_in","cap_out","ring_in","ring_out","storage_in","storage_out","delivery_in","delivery_out"]
                    print("Chose MPS")
                    z_ans = int(kotae())
                    print(str(name_check[z_ans]))

                    time_end =  time.perf_counter()
                    print("mps_check time is {}\n\n".format(time_end-time_sta))

    time_pg_end =  time.perf_counter()
    print("program time is {}".format(time_pg_end-time_pg_sta))

