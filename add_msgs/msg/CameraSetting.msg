#Update date: 2024.09.25 (yeongjun.park)
#CameraSetting.msg
#Deepstream 
uint8 flag            # BIT MASKING
                      # 0b0000 0001 = activation
                      # 0b0000 0010 = type
                      # 0b0000 0100 = uri
                      # 0b0000 1000 = latency
                      # 0b0001 0000 = dropframe
                      # 0b0010 0000 = port
                      # 0b0100 0000 = width
                      # 0b1000 0000 = height

bool activation       # enable=1, unable=0			
uint16 type           # 1: Camera (V4L2)
                      # 2: URI
                      # 3: MultiURI
                      # 4: RTSP
                      # 5: Camera (CSI) (Jetson only)

string uri            # uri=rtsp://192.168.1.10/color			
uint16 latency        # latency=0			
uint16 dropframe      # drop-frame-interval=5			
uint16 port           # rtsp-port = 8554			
uint32 width          # width = 1280			
uint32 height         # height = 720			