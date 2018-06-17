import sys
import numpy as np
import cv2
from math import sqrt

import pyzbar.pyzbar as pyzbar
import pytesseract

def is_plot(img):
   
    width, height = img.shape
    #cv2.imshow('Warped image',img)
    #cv2.waitKey()

    for i in [71, 72, 73]:
        for j in range(0, width):
            if (img[j, i] < 255):
                return True

    return False


def extract_plot(img):

    #preveri je na sliki slucajno prevec crne (recimo QR)
    if (np.mean(img) < 200):
        return None
    
    print("Found a plot in the image!")

    width, height = img.shape

    pointsX = []
    pointsY = []

    #tabela v prvem for je za piksle kjer se bo gledal kot, med 30 in 110 se doda se par tock
    # height je levo->desno, width je gor->dol (PAZI PRI SPREMEMBAH)
    for i in [25, 32, 40, 47, 55, 62, 70, 77, 85, 92, 100, 107, 115]:
        for j in range(0, width):
            #print(img[j, i])
            if (img[j, i] < 255):
                pointsX.append(i)
                pointsY.append(width-j) #width-j je zato ker zacne gledat od zgorej rabimo pa kot od spodej
                break
            else:
                img[j, i] = 0
                
    #izracun kota
    slope = np.polyfit(pointsX, pointsY, 1)[0]
    angle = np.arctan(slope) * 180/np.pi

    # Visualize the image
    #cv2.imshow('Warped image',img)
    #cv2.waitKey()

    return slope


def extract_text(img):
    
    # Pass some options to tesseract
    config = '--psm 13 outputbase nobatch digits'
    
    # Visualize the image we are passing to Tesseract
    #cv2.imshow('Warped image',img)
    #cv2.waitKey()

    # Extract text from image
    text = pytesseract.image_to_string(img, config = config)
    
    # Check and extract data from text
    print('Extracted>>',text)
    
    # Remove any whitespaces from the left and right
    text = text.strip()
    
    # If the extracted text is of the right length
    if len(text)==2:
        x=int(text[0])
        y=int(text[1])
        print('The extracted datapoints are x=%d, y=%d' % (x,y))
        return [x,y]
    else:
        print('The extracted text has is of length %d. Aborting processing' % len(text))

def extract_data(img):

    #sharpening, ki izboljsa zaznavo qr kode
    blur = cv2.GaussianBlur(img, (0, 0), 3)
    cv2.addWeighted(img, 1.5, blur, -0.5, 0, img)
    #print('Ring detected! (hopefully)')
                    
    # Find a QR code in the image
    decodedObjects = pyzbar.decode(img)
    #cv2.imshow('Warped image',img)
    #cv2.waitKey()
    #print(decodedObjects)

    data = None

    img_za_backup = img
    
    if len(decodedObjects) == 1:
        dObject = decodedObjects[0]
        print("Found a QR code in the image!")
        #print("Data: ", dObject.data,'\n')
        data = dObject.data
        tmp = data.split(';') 
        data = []
        for t in tmp:
            tt = map(float,t.split('_'))
            data.append(tt[0])
            data.append(tt[1])
        # Visualize the detected QR code in the image
        points  = dObject.polygon
        if len(points) > 4 : 
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else : 
            hull = points;
            
        ## Number of points in the convex hull
        n = len(hull)
        
        ## Draw the convext hull
        for j in range(0,n):
            cv2.line(img, hull[j], hull[ (j+1) % n], (0,255,0), 2)
            
        #cv2.imshow('Warped image',img)
        #cv2.waitKey()
    else:
        
         # Cut out everything but the numbers/plot
        img = img[125:221,50:195,:]
        
        # Convert the image to grayscale
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Option 1 - use ordinairy threshold the image to get a black and white image
        #ret,img = cv2.threshold(img,100,255,0)

        # Option 1 - use adaptive thresholding
        #img = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,5)
        
        # Use Otsu's thresholding
        ret,img = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        if (is_plot(img)):

            data = extract_plot(img)
            data = [data]
            if(data == [None]):    # None je v primeru ko je prevelik delez slike crn (verjetno QR)
                print("Napaka: Na sliki je verjetno QR ampak ni bil prepoznan")


                img_za_backup = cv2.cvtColor( img_za_backup, cv2.COLOR_RGB2GRAY )
                ret, img_za_backup = cv2.threshold(img_za_backup, 120, 255, 0)
                #cv2.imshow('thresh image',img_za_backup)
                #cv2.waitKey()

                decodedObjects2 = pyzbar.decode(img_za_backup)

                if len(decodedObjects2) == 1:
                    dObject = decodedObjects2[0]
                    print("Found a QR code in the image!")
                    #print("Data: ", dObject.data,'\n')
                    data = dObject.data
                    tmp = data.split(';') 
                    data = []
                    for t in tmp:
                        tt = map(float,t.split('_'))
                        data.append(tt[0])
                        data.append(tt[1])
                    # Visualize the detected QR code in the image
                    points  = dObject.polygon
                    if len(points) > 4 : 
                        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                        hull = list(map(tuple, np.squeeze(hull)))
                    else : 
                        hull = points;
                        
                    ## Number of points in the convex hull
                    n = len(hull)
                    
                    ## Draw the convext hull
                    for j in range(0,n):
                        cv2.line(img, hull[j], hull[ (j+1) % n], (0,255,0), 2)

                    if data == None:
                        return [None]
        else:
            data = extract_text(img)
            #data = "popravi pytesseract"

    return data

def extract_color(img):
    
    mask = cv2.imread('/home/team_lambda/ROS/src/exercise4/scripts/mask.jpg',cv2.IMREAD_GRAYSCALE)

    # Use Otsu's thresholding
    ret,mask = cv2.threshold(mask,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # count non zero for averaging later
    n = cv2.countNonZero(mask)
    #change mask to a 3 channel image 
    mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #apply mask to the hsv image
    hsv_masked = cv2.subtract(mask,hsv)
    hsv_masked = cv2.subtract(mask,hsv_masked)

    avg_h = cv2.sumElems(hsv_masked[:,:,0])[0] / n

    print(avg_h)


    if(avg_h > 130 or avg_h < 15):
        return "red"
    elif(avg_h > 40 and avg_h < 70):
        return "green"
    elif(avg_h > 95 and avg_h < 130):
        return "blue"
    elif(avg_h > 15 and avg_h < 35):
        return "yellow"

    #cv2.imshow('mask',mask)
    #cv2.waitKey()


# extract_data locuje naprej
#img = cv2.imread('/home/lambda/ROS/src/slike/qrrdec.png',cv2.IMREAD_COLOR)
#print(extract_data(img))
#print(extract_color(img))

# read_circle_data(img)
# extract_text(img)
# print(extract_data(img))
# extract_plot(img)
