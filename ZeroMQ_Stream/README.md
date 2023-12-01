# Start the server at 30fps 
python server.py --fps 30

# Start the viewing proxy
python webproxy.py 

# Browse to the viewing portal 
http://192.168.1.140:8080

# Run transforms on the frames (draw dots, and bounding boxes)
python transform.py 

