import urllib.request
import numpy as np
import cv2
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters.detect import get_objects
from pycoral.adapters.common import input_size

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    # convert the input image to the HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # convert the hsv to a binary image by removing any pixels
    # that do not fall within the following HSV Min/Max values
    img_threshold = cv2.inRange(img_hsv, (60, 70, 70), (85, 255, 255))

    # find contours in the new binary image
    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])

    # initialize an empty array of values to send back to the robot
    llpython = [0,0,0,0,0,0,0,0]

    # if contours have been detected, draw them
    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        # record the largest contour
        largestContour = max(contours, key=cv2.contourArea)

        # get the unrotated bounding box that surrounds the contour
        x,y,w,h = cv2.boundingRect(largestContour)

        # draw the unrotated bounding box
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)

        # record some custom data to send back to the robot
        llpython = [1,x,y,w,h,9,8,7]

    # Load the Coral-compiled model
    interpreter = make_interpreter('model_edgetpu.tflite')
    interpreter.allocate_tensors()

    while True:

        # Preprocess the image (resize and normalize for Coral model)
        image_resized = cv2.resize(image, input_size(interpreter))  # Resize to model's input size
        input_tensor = np.expand_dims(image_resized, axis=0)

        # Run inference using Coral
        interpreter.set_tensor(interpreter.get_input_details()[0]['index'], input_tensor)
        interpreter.invoke()
        objs = get_objects(interpreter, 0.4)  # Confidence threshold

        # Process the results (e.g., draw bounding boxes)
        for obj in objs:
            bbox = obj.bbox
            # Draw bounding box on the frame (for visualization)
            cv2.rectangle(image, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255, 0, 0), 2)
            print(f"Detected object {obj.label_id} with confidence {obj.score}")

        # Display the frame (for debugging)
        cv2.imshow('Limelight Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        cv2.destroyAllWindows()

    #return the largest contour for the LL crosshair, the modified image, and custom robot data
    return largestContour, image, llpython
