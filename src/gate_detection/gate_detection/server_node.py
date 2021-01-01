from interfaces.srv import GateLocation   
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import matplotlib.pyplot as plt


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.contours =[]
        self.x_arr = []
        self.gate_arr = []
        self.y_arr = []
        self.x_center=0
        self.y_center=0
        self.block_number=0
        self.srv = self.create_service(GateLocation, 'find_gate_location', self.find_gate_location_callback)  
        
#***********************************Gate_detection_part*******************************************
    def nearGate (self, imgHSV):
        # near gate color detection
        lower = np.array([0, 0, 0])
        upper = np.array([79, 131, 255])
        mask = cv2.inRange(imgHSV, lower, upper)
        Erokerne = np.ones((6, 6), np.uint8) 
        Dikernel = np.ones((3, 3), np.uint8) 
        imgEroded = cv2.erode(mask, Erokerne, iterations=1)
        imgDialated = cv2.dilate(imgEroded, Dikernel, iterations=1)
        print('near')
        return imgDialated


    def farGate (self,imgHSV):
        # far gate color detection
        lower = np.array([0, 0, 0])
        upper = np.array([179, 131, 255])
        mask = cv2.inRange(imgHSV, lower, upper)
        Erokerne = np.ones((6, 6), np.uint8) 
        Dikernel = np.ones((3, 3), np.uint8)  
        imgEroded = cv2.erode(mask, Erokerne, iterations=1)
        imgDialated = cv2.dilate(imgEroded, Dikernel, iterations=8)
        # to remove image noise
        print('far')
        return imgDialated



    def getContours(self,imgHSV, imgContour):
        self.hight=imgHSV.shape[0]
        self.width=imgHSV.shape[1]
        self.vlines=[int(self.width/3),int(2*self.width/3),int(self.width) ]
        self.hlines = [int(self.hight / 3), int(2 * self.hight / 3),int (self.hight)]

        self.x1=self.vlines[0]
        self.x2=self.vlines[1]
        self.y1=self.hlines[1]
        self.y2=self.hlines[0]

        for line in self.vlines:
            cv2.line(imgContour, (line, 0), (line, self.hight), (250,250, 250), thickness=2)
        for line in self.hlines:
            cv2.line(imgContour, (0, line), (self.width, line), (250, 250, 250), thickness=2)

        flage = True
        Nmask = self.nearGate(imgHSV)
        # contour detection
        retrieval_mod = cv2.RETR_EXTERNAL
        Ncontours, Nhierarchy = cv2.findContours(Nmask, retrieval_mod, cv2.CHAIN_APPROX_NONE)
        if Ncontours:
            maxArea=cv2.contourArea(Ncontours[0])
            x=0
            for cnt in Ncontours:
                area = cv2.contourArea(cnt)
                if area > maxArea:
                    maxArea=area
            if maxArea>700:
                x=1
                # print(maxArea)

        # if there is no gate detected (may be far)
        if len(Ncontours) <4 or x==0:
            Fmask = self.farGate(imgHSV)
            Fcontours, Fhierarchy = cv2.findContours(Fmask, retrieval_mod, cv2.CHAIN_APPROX_NONE)
            self.contours = Fcontours

            # not near or far (no gate)
            if len(self.contours) ==0:
                flage = False
        else:
            self.contours = Ncontours

        if flage:
            for cnt in self.contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    curveLength = cv2.arcLength(cnt, True)
                    approx_corner_points = cv2.approxPolyDP(cnt, 0.02 * curveLength, True)
                    Num_objCorners = len(approx_corner_points)

                    if Num_objCorners > 2 and Num_objCorners < 10:

                        x, y, w, h = cv2.boundingRect(approx_corner_points)
                        self.x_arr.append(x), self.gate_arr.append(y + h ), self.y_arr.append(y)
                        # cv2.rectangle(imgContour, (x, y), (x + w, y + h), (200, 100, 255), 15)
                        # cv2.drawContours(imgContour, cnt, -1, (0, 255, 100), 5)
                    else:
                        pass
            if self.gate_arr:
                gate_hight1 = max(self.gate_arr)
                indx1 = self.gate_arr.index(max(self.gate_arr))
                self.gate_arr[indx1] = 0
                indx2 = self.gate_arr.index(max(self.gate_arr))

                if abs(self.x_arr[indx1]-self.x_arr[indx2])>500 :
                    if abs(gate_hight1 - max(self.gate_arr))<400:
                        if self.x_arr[indx1] > self.x_arr[indx2]:
                            cv2.rectangle(imgContour, (self.x_arr[indx1], self.y_arr[indx1] - 50), (self.x_arr[indx2], gate_hight1), (100, 255, 100),
                                        20)
                        else:
                            cv2.rectangle(imgContour, (self.x_arr[indx2], self.y_arr[indx1] - 50), (self.x_arr[indx1], gate_hight1), (100, 255, 100),
                                        20)

                        cv2.putText(imgContour, 'Gate',
                                    (self.x_arr[indx1], self.y_arr[indx1] - 100), cv2.Formatter_FMT_MATLAB, 5,
                                    (100, 255, 100), 10)

                        # print(self.x_arr[indx1],self.x_arr[indx2],self.y_arr[indx1],gate_hight1 )

                        self.x_center= int(0.5*self.x_arr[indx1]+0.5*self.x_arr[indx2])
                        self.y_center= int(0.5*self.y_arr[indx1]+0.5*gate_hight1)

                    # to detect the narrower pass (bonus)
                    self.y_arr[indx1] = 0
                    self.y_arr[indx2] = 0
                    bindx = self.y_arr.index(max(self.y_arr))

                    if max(self.x_arr) < self.x_arr[bindx] + 120 or min(self.x_arr) > self.x_arr[bindx] - 120:
                        pass
                    elif abs(self.x_arr[bindx] - self.x_arr[indx1]) > abs(self.x_arr[bindx] - self.x_arr[indx2]):
                        cv2.rectangle(imgContour, (self.x_arr[bindx], self.y_arr[bindx]), (self.x_arr[indx2], gate_hight1), (200, 0, 255), 10)
                        cv2.putText(imgContour, 'bouns entry',
                                    (self.x_arr[bindx], self.y_arr[bindx] - 50), cv2.Formatter_FMT_MATLAB, 5,
                                    (000, 0, 255), 10)

                        self.x_center= int(0.5*self.x_arr[bindx]+0.5*self.x_arr[indx2])
                        self.y_center= int(0.5*self.y_arr[bindx]+0.5*gate_hight1  )          
                    else:
                        cv2.rectangle(imgContour, (self.x_arr[bindx], self.y_arr[bindx]), (self.x_arr[indx1], gate_hight1), (200, 0, 255), 10)
                        cv2.putText(imgContour, 'bouns entry',
                                    (self.x_arr[bindx], self.y_arr[bindx] - 50), cv2.Formatter_FMT_MATLAB, 5,
                                    (000, 0, 255), 10)

                        self.x_center=int( 0.5*self.x_arr[bindx]+0.5*self.x_arr[indx1])                    

#*******************************************************************************************

    def block(self,x,y):
        if x==0 and y==0:
            self.block_number=0  

        elif x <self.x1:
            if y<self.y2:
                self.block_number=1
            elif y >self.y2 and y<self.y1:
                self.block_number=4
            else:
                self.block_number=7
        elif x >self.x1 and x <self.x2 :
            if y<self.y2:
                self.block_number=2
            elif y >self.y2 and y<self.y1:
                self.block_number=5
            else:
                self.block_number=8

        elif x >self.x2 :
            if y<self.y2:
                self.block_number=3
            elif y >self.y2 and y<self.y1:
                self.block_number=6
            else:
                self.block_number=9 
        else:
            pass

           

    def find_gate_location_callback(self, request, response):
        #uncomment the default path to launch with launch file and comment (path = request.image_path)
        path = '/home/fatma/AUV_ws/src/frame112.jpg'
        # path = request.image_path
        img = cv2.imread(path)
        imgContour = img.copy()
        img = cv2.medianBlur(img, 23)
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        self.getContours(imgHSV, imgContour)
        self.block(self.x_center,self.y_center)

        if self.block_number==1:
            response.gate_location='Move one block up, then one block left'
        elif self.block_number==2:
            response.gate_location='Move one block up'
        elif self.block_number==3:
            response.gate_location='Move one block up, then one block right'
        elif self.block_number==4:
            response.gate_location='Move one block left'
        elif self.block_number==5:
            response.gate_location='Move forward, gate exist here'
        elif self.block_number==6: 
            response.gate_location='Move one block right'
        elif self.block_number==7:
            response.gate_location='Move one block down, then one block left'
        elif self.block_number==8:
            response.gate_location='Move one block down'
        elif self.block_number==9: 
            response.gate_location= 'Move one block down, then one block right' 
        else:
            response.gate_location='gate not exist here'                          
        self.get_logger().info('Incoming request\na:%s' % (request.image_path))
        print(response.gate_location,self.block_number)

        cv2.putText(imgContour, response.gate_location+',block'+str(self.block_number),
                                    (int(self.width/10), int(self.hight/10)), cv2.Formatter_FMT_MATLAB, 4,
                                    (0, 0, 0), 5)
        imgplot = plt.imshow(imgContour)
        plt.show()                                                    
        return response   
        
def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()