import cv2
import numpy as np

team_color = "RED"

blueLower1 = np.array([85, 150, 0])
blueUpper1 = np.array([138, 255, 255])

redLower1 = np.array([0, 100, 20])
redUpper1 = np.array([10, 255, 255])
 
redLower2 = np.array([160,100,20])
redUpper2 = np.array([179,255,255])


bumperThres = 420
#upperThres = 100

old_pts = [None,None]
max_balls = 5


r_threshold = [15,80,15]
# def cvimage_to_pygame(image):
#     return pygame.image.frombuffer(image.tostring(), image.shape[1::-1],"RGB")


# def pygame_to_cvimage(surface):
#     view = pygame.surfarray.array3d(surface)
#     view = view.transpose([1,0,2])
#     #view = cv2.cvtColor(view,cv2.COLOR_RGB2BGR)
#     return view


# def cvimage_to_pygame(image):
#     return pygame.image.frombuffer(image.tostring(), image.shape[1::-1],"RGB")


# def pygame_to_cvimage(surface):
#     view = pygame.surfarray.array3d(surface)
#     view = view.transpose([1,0,2])
#     #view = cv2.cvtColor(view,cv2.COLOR_RGB2BGR)
#     return view


def generate_circles(frame):
    #frame = imutils.resize(frame,width=600)
    frame = cv2.resize(frame,(600,450))
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    if team_color == "RED":
        lower_mask = cv2.inRange(hsv, redLower1, redUpper1)
        upper_mask = cv2.inRange(hsv, redLower2, redUpper2)
        mask = lower_mask + upper_mask
    else: #color is blue
        mask = cv2.inRange(hsv, blueLower1,blueUpper1)

    mask = cv2.erode(mask, None, iterations=4)
    mask = cv2.dilate(mask, None, iterations=4)
    #cv2.imshow('mask',mask)
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, .4, 60 , param1=200, param2=14, minRadius=r_threshold[0], maxRadius=r_threshold[1]+r_threshold[2])
    valid_circles = []
    if circles is not None:
	    # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            target_r = r_threshold[0] + (y/(bumperThres+0.01))*(r_threshold[1]-r_threshold[0])
            if y < bumperThres and r > target_r -r_threshold[2] and r < target_r + r_threshold[2]:
                valid_circles.append((round((x-frame.shape[1]/2)/(frame.shape[1]/2),3),round(y/(bumperThres+0.01),3),r))
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    cv2.line(frame,(0,bumperThres),(frame.shape[1],bumperThres),(0,255,0),2)
    return [valid_circles,frame]



