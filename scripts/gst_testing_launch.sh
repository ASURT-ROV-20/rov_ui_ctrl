gst-launch-1.0 v4l2src ! video/x-raw,width=640,height=480 ! jpegenc ! rtpjpegpay ! multiudpsink clients=127.0.0.1:5000,127.0.0.1:5001,127.0.0.2:5002
