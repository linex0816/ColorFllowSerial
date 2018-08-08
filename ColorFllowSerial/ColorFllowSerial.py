from collections import  deque  
import numpy as np  
import time  
#import imutils  
import cv2  
import serial
import string
import binascii
import time
import struct  


#打开串口
s=serial.Serial('com6',9600)
if s.isOpen():
    s.close()
s.open()
#舵机归位
d=bytes.fromhex('55 55 0B 03 02 E8 03 01 DC 05 00 E8 03') 
s.write(d)
time.sleep(1.3)
posx = 1500  #定义舵机x初始位置
posy = 1000  #定义舵机y初始位置

distancex = 5  # 舵机x步进
distancey = 5  # 舵机y步进

KPx = 1
KPy = 1
KDx = 0
KDy = 0
errorx = 0
lasterrorx = 0
errory = 0
lasterrory = 0
timeo = 0
errorarrayx =[0,1,2]
errorarrayy =[0,1,2]
#pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)] 舵机位置式PID

#设定红色阈值，HSV空间  
redLower = np.array([170, 100, 100])  
redUpper = np.array([179, 255, 255])  
#初始化追踪点的列表  
mybuffer = 16  
pts = deque(maxlen=mybuffer)  
counter = 0  
#打开摄像头  
camera = cv2.VideoCapture(0)  

#等待两秒  
time.sleep(3)  
#遍历每一帧，检测红色瓶盖  
while True: 
    
    #读取帧  
    (ret, frame) = camera.read() 

    #判断是否成功打开摄像头  
    if not ret:  
        #print 'No Camera'  
        break  

    #得到摄像头图像的宽和高
    frameWidth = camera.get(3)
    frameHigh = camera.get(4)
    halfFrameWidth = frameWidth / 2
    halfFrameHigh = frameHigh / 2

    #frame = imutils.resize(frame, width=600)  
    #转到HSV空间  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
    #根据阈值构建掩膜  
    mask = cv2.inRange(hsv, redLower, redUpper)  
    #腐蚀操作  
    mask = cv2.erode(mask, None, iterations=2)  
    #膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点  
    mask = cv2.dilate(mask, None, iterations=2)  
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  
    #初始化瓶盖圆形轮廓质心  
    center = None  
    #如果存在轮廓  
    if len(cnts) > 0:  
        #找到面积最大的轮廓  
        c = max(cnts, key = cv2.contourArea)  
        #确定面积最大的轮廓的外接圆  
        ((x, y), radius) = cv2.minEnclosingCircle(c)  
        #计算轮廓的矩  
        M = cv2.moments(c)  
        #计算质心  
        center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])) 

        if timeo > 2: #下面的代码是用来计算标记物与画面中点的像素差，采用了位置式PID控制算法计算出像素差，在通过判断像素差定义舵机的步进值。下一步是考虑怎么实现将PID值直接作为舵机的位置。
            errorarrayx.append(halfFrameWidth - center[0])
            errorarrayx.pop(0)
            lasterrorx =errorarrayx[2]
            errorx = errorarrayx[1]
            disinchx =abs(KPx * lasterrorx + KDx * (lasterrorx - errorx))
            if 0 < disinchx < 80:
                distancex = 5
            elif 81 < disinchx < 160:
                distancex = 10
            elif 161 < disinchx < 240:
                distancex = 25
            elif disinchx > 241:
                distancex =40         
                
            errorarrayy.append(halfFrameHigh - center[1])
            errorarrayy.pop(0)
            lasterrory =errorarrayy[2]
            errory = errorarrayy[1]
            disinchy =abs(KPy * lasterrory + KDy * (lasterrory - errory))
            if 0 < disinchy < 80:
                distancey = 5
            elif 81 < disinchy < 160:
                distancey = 10
            elif 161 < disinchy < 240:
                distancey = 25
            elif disinchy > 241:
                distancey =40  
            #print(disinchx,disinchy,distancex,distancey)
        timeo +=1

        if int(center[0]) < int(halfFrameWidth - 20):  #判断center X轴得到的值是否在画面中点的左边
            if posx >= (500 + 10):   #判断舵机是否移动到了右边旋转角度极限
                posx += distancex    #如果没有则增加步进                
            else : posx =1500

        elif int(center[0]) > int(halfFrameWidth + 20): #判断center X轴得到的值是否在画面中点的右边
            if posx <= (2500 -10):  #判断舵机是否移动到了左边旋转角度极限
                posx -= distancex   #如果没有则减少步进                
            else : posx = 1500          

        if int(center[1]) < int(halfFrameHigh - 20) :    #判断center Y轴得到的值是否在画面中点的下边
            if posy >= (500 + 10):                       #判断舵机是否移动到了下边旋转角度极限
                posy += distancey
            else : posy =1000
        elif int(center[1]) > int(halfFrameHigh + 20):       #判断center Y轴得到的值是否在画面中点的上边
            if posy <= (2500 - 10):                          #判断舵机是否移动到了上边旋转角度极限
                posy -= distancey
            else : posy =1000

        dXbyte = [ hex(i) for i in struct.pack('<h',int(posx))]  #将舵机X位置转换为十六进制
        dXStrL = dXbyte[0][2:].zfill(2)                     #取十六进制舵机位置的 低位
        dXStrH = dXbyte[1][2:].zfill(2)                     #取十六进制舵机位置的 高位
        dYbyte = [ hex(i) for i in struct.pack('<h',int(posy))]
        dYStrL = dYbyte[0][2:].zfill(2)
        dYStrH = dYbyte[1][2:].zfill(2)   

        str='55 55 0B 03 02 32 00 01 '+ dXStrL + ' ' + dXStrH + " 00 " + dYStrL + ' ' + dYStrH       #编成舵机控制板串口码
        #str='55 55 0B 03 02 32 00 01 '+ dXStrL + ' ' + dXStrH + " 00 E8 03"       #编成舵机控制板串口码 固定上舵机
        d=bytes.fromhex(str) 
        s.write(d)     

        #只有当半径大于5时，才执行画图  
        if radius > 5:  
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)  
            cv2.circle(frame, center, 5, (0, 0, 255), -1)  
            #把质心添加到pts中，并且是添加到列表左侧  
            pts.appendleft(center) 

            cv2.line(frame,(310,240),(330,240),(192,192,192),1) #画面中间显示十字的横线
            cv2.line(frame,(320,230),(320,250),(192,192,192),1) #画面中间显示十字的纵线

    else:#如果图像中没有检测到瓶盖，则清空pts，图像上不显示轨迹。  
        pts.clear()  
      
    for i in range(1, len(pts)):  
        if pts[i - 1] is None or pts[i] is None:  
            continue  
        #计算所画小线段的粗细  
        thickness = int(np.sqrt(mybuffer / float(i + 1)) * 2.5)  
        #画出小线段  
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)  
        #判断移动方向  
        if counter >= 10 and i == 1 and len(pts) >= 10:  
            dX = pts[-10][0] - pts[i][0]  
            dY = pts[-10][1] - pts[i][1]  
            (dirX, dirY) = ("", "")  

            if np.abs(dX) > 20:  
                dirX = "East" if np.sign(dX) == 1 else "West"                 

            if np.abs(dY) > 20:  
                dirY = "North" if np.sign(dY) == 1 else "South"  
              
            if dirX != "" and dirY != "":  
                direction = "{}-{}".format(dirY, dirX)  
            else:  
                direction = dirX if dirX != "" else dirY  
          
            cv2.putText(frame, direction, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2,   
                        (0, 255, 0), 3)  
            cv2.putText(frame, "dx: {}, dy: {}".format(dX, dY), (10, frame.shape[0] - 10),   
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)  
              
    cv2.imshow('Frame', frame)  
    
    #键盘检测，检测到esc键退出  
    k = cv2.waitKey(1)&0xFF  
    counter += 1  
    if k == 27:  
        break  
#摄像头释放  
camera.release() 
#关闭串口
s.close()
#销毁所有窗口  
cv2.destroyAllWindows() 




