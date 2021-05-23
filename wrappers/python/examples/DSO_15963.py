import pyrealsense2 as rs
import time
import math

context = rs.context()
all_frames = {}
frames_num = []
def callback(frame):
    f_time = frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp)
    f_num = frame.get_frame_metadata(rs.frame_metadata_value.frame_counter)
    all_frames[f_num] = f_time
    frames_num.append(f_num)

iterations = 1
frames_per_iteration = 5

deviceList = context.query_devices()
dev = deviceList.front()
sensorsList = dev.query_sensors()
color_sensor = next(s for s in sensorsList if s.get_info(rs.camera_info.name) == 'RGB Camera')

pipe = rs.pipeline()
color_sensor.set_option(rs.option.auto_exposure_priority, 0)
cfg = rs.config()


cfg.enable_stream(rs.stream.color, 0, 0, rs.format.rgb8, 0) # RGB8 and RGBA8
cfg.enable_stream(rs.stream.color, 0, 0, rs.format.rgba8, 0)

pipe.start(cfg, callback)
time.sleep(10) # wait 10 seconds
pipe.stop()

# analysis

#for num_, timestamp in all_frames.items() :
prev_fnum = frames_num[0]
for i in range(1, len(frames_num)):
    fnum = frames_num[i]
    print("Frame number: ", fnum, " - Time Stamp: ", all_frames[fnum])
    if fnum > prev_fnum+1 :
        print("DROP : drop detected after frame :", prev_fnum, " - following frame: ", fnum)
    prev_fnum = fnum

