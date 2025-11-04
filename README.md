# Motion-Detection
This MATLAB program detects and tracks motion from a video file or live camera. It uses background subtraction, frame differencing, and blob analysis to identify moving objects, displaying them with bounding boxes. Users can adjust sensitivity and blob size through a simple GUI.


This MATLAB program implements an Enhanced Motion Detection and Tracking System with a graphical user interface (GUI). It allows users to choose between a video file or a live camera feed as the input source.

The system processes each video frame to detect motion using background subtraction and frame differencing techniques. Detected moving objects are refined using morphological filtering and blob analysis, and are highlighted with green bounding boxes in real time.

Users can adjust the detection sensitivity and minimum blob area using sliders in the GUI to control how responsive the motion detection is. The interface displays both the original video and the processed output side-by-side for easy comparison.
