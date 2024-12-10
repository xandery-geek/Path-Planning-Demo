#!/usr/lib/python3 
import cv2 
import numpy as np 

def covert(src):

    b = cv2.split(src)[0]
    g = cv2.split(src)[1]
    r = cv2.split(src)[2]

    des = cv2.merge([r,g,b])

    return des

def main():

    cv2.namedWindow("src", cv2.WINDOW_NORMAL)
    cv2.namedWindow("des", cv2.WINDOW_NORMAL)

    src = cv2.imread("./power_blue.png")
    des = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    #des = covert(src)
    cv2.imwrite("./power_gray.png", des, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
    
    cv2.imshow("src", src)
    cv2.imshow("des", des)

    while cv2.waitKey() != 27:
        pass

if __name__=="__main__":
    main()



