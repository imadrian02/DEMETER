import numpy as np
import imutils
from ultralytics import YOLO
import supervision as sv
from greenonbrown import GreenOnBrown
from xgantry import *
from videocapture import *


# Detection init
start_time = time.time()
model = YOLO(r'C:\Users\adria\DEMETER\model\best.pt')
detector = GreenOnBrown(algorithm='exg')

tracker = sv.ByteTrack()
box_annotator = sv.BoundingBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# w, h = 1920, 1080
cap = VideoCapture(0)

# cap.set(cv2.CAP_PROP_FRAME_WIDTH, value=w)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, value=h)

# w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
# h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
# fps = cap.get(cv2.CAP_PROP_FPS)

# Test vid: 1280x720
# print(f'[Update] Machine initiated with frame w: {w}, h: {h}, fps: {fps}')
# print()

# Actuator init
gantry1 = XGantry()
gantry2 = XGantry()
frame_gantry_ratio = 0.140625

# port, baud_rate = '/dev/ttyUSB0', 115200
# arduino = serial.Serial(port, baud_rate, timeout=1)
#
# port, baud_rate = '/dev/ttyACM0', 115200
# arduino2 = serial.Serial(port, baud_rate, timeout=1)

counter = 0
assigned = []
completed = []

prev_pos1, prev_pos2 = None, None

treat_track_boundary = 320


def get_centroid(x1, y1, x2, y2):
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    return cx, cy


def encode_to_arduino(g1=None, g2=None):
    if g1 is not None and g2 is None:
        msg = 'a' + str(g1)
    elif g1 is None and g2 is not None:
        msg = 'b' + str(g2)
    elif g1 is not None and g1 is not None:
        msg = 'a' + str(g1) + ',' + 'b' + str(g2)

    print('[Comm] Sending msg to Arduino: ' + msg)
    # arduino.write(msg.encode())


def encode_to_arduino2():
    l1 = gantry1.laser.status

    msg = f"a0,b0:{l1},0"

    print('[Comm] Sending msg to Arduino2: ' + msg)
    # arduino2.write(msg.encode())
    time.sleep(0.01)


laser_state = 0
offset = 1

while True:
    frame = cap.read()

    update1, pos1 = False, None
    update2, pos2 = False, None

    # if ret:
    #     # fps setting for video
    #     # counter += 1
    #     # cap.set(cv2.CAP_PROP_POS_FRAMES, counter)
    # else:
    #     break

    # Frame resize
    frame = imutils.resize(frame, width=640, height=480)
    # cv2.line(frame, [0, treat_track_boundary], [640, treat_track_boundary], [255, 255, 255], 1)

    # Crop detect
    results = model(frame, conf=0.9, verbose=False)
    crop_frame = results[0].plot()
    crop_boxes = results[0].boxes.xyxy.cpu().numpy()

    # Masking
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for crop_box in crop_boxes:
        x1, y1, x2, y2 = crop_box
        m = 12
        cv2.rectangle(mask, (int(x1) - m, int(y1) - m), (int(x2) + m, int(y2) + m), (255, 255, 255), -1)

    masked_frame = frame.copy()
    masked_frame[mask == 255] = [0, 0, 0]

    # Weed Detect
    results = detector.inference(masked_frame)

    # Tracking
    detections = sv.Detections.from_azure_analyze_image(results)
    detections = tracker.update_with_detections(detections)

    # Machine algo
    miss1, miss2 = True, True
    prio_id, prio_x, prio_y = 0, 0, 0

    for tracker_id, xyxy in zip(detections.tracker_id, detections.xyxy):
        x1, y1, x2, y2 = xyxy.tolist()
        x, y = get_centroid(x1, y1, x2, y2)

        if tracker_id in assigned:
            if gantry1.status:
                if tracker_id == gantry1.job["Target"]:
                    miss1 = False
                    gantry1.job["TargetX"] = x
                    gantry1.job["TargetY"] = y

            if gantry2.status:
                if tracker_id == gantry2.job["Target"]:
                    miss2 = False
                    gantry2.job["TargetX"] = x
                    gantry2.job["TargetY"] = y

        else:
            if prio_y < y < treat_track_boundary:
                prio_id = tracker_id
                prio_x = x
                prio_y = y

    treating1 = threading.Thread(target=gantry1.treat, args=(20, 12))
    treating2 = threading.Thread(target=gantry2.treat, args=(7,))

    # Job assign, Gantry 1
    if not gantry1.status:
        if prio_id != 0 and prio_id not in assigned:
            assigned.append(prio_id)
            gantry1.updateJob(target=prio_id, targetx=prio_x, targety=prio_y)

            update1 = True
            pos1 = str(int((gantry1.job["TargetX"] * frame_gantry_ratio) + offset))
            gantry1.moveTo(pos1)

        else:
            print('[Update] Gantry 1 available')

    else:
        if miss1:
            if gantry1.job["LastSeen"] < 5:
                gantry1.job["LastSeen"] += 1
                last = gantry1.job["LastSeen"]
                print(f"[Update] Gantry 1 target missing for {last} frames")

            else:
                print('[Update] Job aborted, gantry 1 will now reset')
                gantry1.reset()

        else:
            gantry1.job["LastSeen"] = 0

            track_id = gantry1.job["Target"]

            pos1 = int((gantry1.job["TargetX"] * frame_gantry_ratio) + offset)

            if gantry1.job["TargetY"] > treat_track_boundary and not gantry1.job["Treat"]:
                gantry1.job["Treat"] = True
                treating1.start()

            if pos1 != prev_pos1:
                update1 = True
                pos1 = str(pos1)
                print(f'[Action] Gantry 1 tracking target {track_id} at x: {pos1}')
                print(gantry1.job)

    # Job assign, Gantry 2
    if not gantry2.status:
        if prio_id != 0 and prio_id not in assigned:
            assigned.append(prio_id)
            gantry2.updateJob(target=prio_id, targetx=prio_x, targety=prio_y)

            update2 = True
            pos2 = str(int((gantry2.job["TargetX"] * frame_gantry_ratio)) + offset)
            gantry2.moveTo(pos2)

        else:
            print('[Update] Gantry 2 available')

    else:
        if miss2:
            if gantry2.job["LastSeen"] < 5:
                gantry2.job["LastSeen"] += 1
                last = gantry2.job["LastSeen"]
                print(f"[Update] Gantry 2 target missing for {last} frames")

            else:
                print('[Update] Job aborted, gantry 2 will now reset')
                gantry2.reset()

        else:
            gantry2.job["LastSeen"] = 0
            track_id = gantry2.job["Target"]

            pos2 = int((gantry2.job["TargetX"] * frame_gantry_ratio) + offset)

            if gantry2.job["TargetY"] > treat_track_boundary and not gantry2.job["Treat"]:
                gantry2.job["Treat"] = True
                treating2.start()

            elif pos2 != prev_pos1:
                update2 = True
                pos2 = str(pos2)
                print(f'[Action] Gantry 2 tracking target {track_id} at x: {pos2}')
                print(gantry2.job)

    # Serial communication
    if update1 and update2:
        encode_to_arduino(g1=pos1, g2=pos2)
    elif update1 and not update2:
        encode_to_arduino(g1=pos1)
    elif not update1 and update2:
        encode_to_arduino(g2=pos2)

    time.sleep(0.01)

    # Laser toggle
    if gantry1.laser.status:
        encode_to_arduino2()
        laser_state = gantry1.laser.status

    if laser_state != gantry1.laser.status:
        encode_to_arduino2()
        laser_state = gantry1.laser.status

    # Display output
    labels = [f"#{tracker_id}" for class_id, tracker_id in zip(detections.class_id, detections.tracker_id)]

    weed_frame = box_annotator.annotate(frame.copy(), detections=detections)
    weed_frame = label_annotator.annotate(weed_frame, detections=detections, labels=labels)
    cv2.imshow("Output", weed_frame)

    # For video testing
    # time.sleep(0.5)

    print()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print(f'[Summary] Session completed in {time.time() - start_time} seconds,\nassigned: {len(assigned)}, completed: {(gantry1.complete + gantry2.complete)}')
encode_to_arduino(g1=0, g2=0)
gantry1.laser.status = 0
encode_to_arduino2()
cv2.destroyAllWindows()
