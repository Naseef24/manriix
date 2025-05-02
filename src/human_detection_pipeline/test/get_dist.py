import pyrealsense2 as rs
import numpy as np
import cv2

# Load the pre-trained MobileNet-SSD model
net = cv2.dnn.readNetFromCaffe('deploy.prototxt', 'mobilenet_iter_73000.caffemodel')

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Prepare the image for object detection
        (h, w) = color_image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(color_image, (300, 300)), 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()

        # Loop over the detections
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            # Filter out weak detections
            if confidence > 0.5:
                idx = int(detections[0, 0, i, 1])

                # Check if the detected object is a person (class index 15 in COCO dataset)
                if idx == 15:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # Draw the bounding box
                    cv2.rectangle(color_image, (startX, startY), (endX, endY), (0, 255, 0), 2)

                    # Calculate the center of the bounding box
                    centerX = int((startX + endX) / 2)
                    centerY = int((startY + endY) / 2)

                    # Get the depth value at the center of the bounding box
                    depth = depth_frame.get_distance(centerX, centerY)

                    # Display the coordinates and distance
                    label = f"X: {centerX}, Y: {centerY}, Distance: {depth:.2f}m"
                    cv2.putText(color_image, label, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Show the output image
        cv2.imshow('RealSense', color_image)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()
