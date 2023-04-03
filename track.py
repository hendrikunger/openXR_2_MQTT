
import ctypes
from ctypes import cast, byref
import time
import xr
import math
from typing import List
from pynput import keyboard
import paho.mqtt.client as mqtt
import json
import pickle
import os
import numpy as np

mqtt_broker = "broker.hivemq.com"
#mqtt_broker = "10.54.129.47"
MQTT_BASE_TOPIC ="EDF/BP/machines/"


keep_going = True
calibrate = [False,False,False]
calibration_file_data = {"TopLeft": xr.Posef(), "TopRight": xr.Posef(), "BottomLeft": xr.Posef()}

class Math(object):
    class Pose(object):
        @staticmethod
        def identity():
            t = xr.Posef()
            assert t.orientation.w == 1
            return t

        @staticmethod
        def translation(translation: List[float]):
            t = Math.Pose.identity()
            t.position[:] = translation[:]
            return t

        @staticmethod
        def rotate_ccw_about_y_axis(radians: float, translation: List[float]):
            t = Math.Pose.identity()
            t.orientation.x = 0
            t.orientation.y = math.sin(radians * 0.5)
            t.orientation.z = 0
            t.orientation.w = math.cos(radians * 0.5)
            t.position[:] = translation[:]
            return t
        123


#Detect Keys (for exit)
def on_press(key):
    global keep_going
    global calibrate
    global calibration_file_data
    try:
        if key.char == "1": #1 Key
            if(not calibrate[1] and not calibrate[2]):
                print("Calibrating TopLeft")
                calibrate[0] = True # Calibrate TopLeft
                calibration_file_data["TopLeft"] = xr.Posef()
            return
        if key.char == "2": #2 Key
            if(not calibrate[0] and not calibrate[2]):
                print("Calibrating TopRight")
                calibrate[1] = True # Calibrate TopRight
                calibration_file_data["TopRight"] = xr.Posef()
            return
        if key.char == "3": #3 Key
            if(not calibrate[0] and not calibrate[1]):
                print("Calibrating BottomLeft")
                calibrate[2] = True # Calibrate BottomLeft
                calibration_file_data["BottomLeft"] = xr.Posef()
            return
        
        if key.char == "c":
            print("Calibrating Position")
            calibrate = True
            calibration_file_data = {}
            return
        if key.char == "l":     
            path = os.path.join(os.path.realpath(os.path.dirname(__file__)),"cal.json")
            if os.path.isfile(path):
                with open(path, "r" ) as f:
                    cal = pickle.load(f)
                    print(cal, type(cal))
            else:
                print("No calibration file found")
            return

        #print('alphanumeric key {0} pressed'.format(key.char))

    except AttributeError:
        if key == keyboard.Key.esc:
            keep_going = False
            print("Stopping")
            return

        print('special key {0} pressed'.format(
            key))

listener = keyboard.Listener(on_press=on_press)
listener.start()


#MQTT Stuff
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("EDF/BP/cmd")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global calibrate
    #print(msg.topic+" "+str(msg.payload))
    calibrate = True
    pass

mqtt_client = mqtt.Client()


mqtt_client = mqtt.Client(client_id="Tracker")
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker, 1883)
mqtt_client.loop_start()


def recalculateTransforms(calibration_file_data, save=False):
    calibration = None
    if save:
        path = os.path.realpath(os.path.dirname(__file__))
        with open( path+"\cal.p", "wb" ) as f:
            pickle.dump(calibration_file_data, f)
        print("Saved Calibration File")
    return calibration



# ContextObject is a high level pythonic class meant to keep simple cases simple.
with xr.ContextObject(
    instance_create_info=xr.InstanceCreateInfo(
        enabled_extension_names=[
            # A graphics extension is mandatory (without a headless extension)
            xr.KHR_OPENGL_ENABLE_EXTENSION_NAME,
            #xr.MND_HEADLESS_EXTENSION_NAME,   #not working at the moment
            xr.extension.HTCX_vive_tracker_interaction.NAME,
        ],
    ),
) as context:
    instance = context.instance
    session = context.session

    # Save the function pointer
    enumerateViveTrackerPathsHTCX = cast(
        xr.get_instance_proc_addr(
            instance,
            "xrEnumerateViveTrackerPathsHTCX",
        ),
        xr.PFN_xrEnumerateViveTrackerPathsHTCX
    )



    # Create the action with subaction path
    # Role strings from
    # https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#XR_HTCX_vive_tracker_interaction
    role_strings = [
        "handheld_object",
        "left_foot",
        "right_foot",
        "left_shoulder",
        "right_shoulder",
        "left_elbow",
        "right_elbow",
        "left_knee",
        "right_knee",
        "waist",
        "chest",
        "camera",
        "keyboard",
    ]
    role_path_strings = [f"/user/vive_tracker_htcx/role/{role}" for role in role_strings]
    role_paths = (xr.Path * len(role_path_strings))(
        *[xr.string_to_path(instance, role_string) for role_string in role_path_strings],
    )
    pose_action = xr.create_action(
        action_set=context.default_action_set,
        create_info=xr.ActionCreateInfo(
            action_type=xr.ActionType.POSE_INPUT,
            action_name="tracker_pose",
            localized_action_name="Tracker Pose",
            count_subaction_paths=len(role_paths),
            subaction_paths=role_paths,
        ),
    )
    # Describe a suggested binding for that action and subaction path
    suggested_binding_paths = (xr.ActionSuggestedBinding * len(role_path_strings))(
        *[xr.ActionSuggestedBinding(
            pose_action,
            xr.string_to_path(instance, f"{role_path_string}/input/grip/pose"))
          for role_path_string in role_path_strings],
    )
    xr.suggest_interaction_profile_bindings(
        instance=instance,
        suggested_bindings=xr.InteractionProfileSuggestedBinding(
            interaction_profile=xr.string_to_path(instance, "/interaction_profiles/htc/vive_tracker_htcx"),
            count_suggested_bindings=len(suggested_binding_paths),
            suggested_bindings=suggested_binding_paths,
        )
    )

    #Load calibration
    path = os.path.join(os.path.realpath(os.path.dirname(__file__)),"cal.p")
    if os.path.isfile(path):
        with open(path, "rb" ) as f:
            calibration_file_data = pickle.load(f)
            print(calibration_file_data, type(calibration_file_data))
    else:
        print("No calibration file found")

    calibration = recalculateTransforms(calibration_file_data)

    file = calibration_file_data.get("TopLeft", xr.Posef())
    file_translation = file.position

    calpos =  Math.Pose.translation([file.position.z,file.position.x,file.position.y]) 
    
    calpos.position = xr.Vector3f(0.0, -file.position.x, file.position.y)
    #calpos.orientation = file.orientation  #xr.Quaternionf(0.0, 0.0, 0.0, 1.0)
    #calpos.position = xr.Vector3f(0.0,0.0,0.0)    #x=z/y=x/z=y
    calpos = xr.Posef()
    print(calpos)
    

    # Create action spaces for locating trackers in each role
    tracker_action_spaces = (xr.Space * len(role_paths))(
        *[xr.create_action_space(
            session=session,
            create_info=xr.ActionSpaceCreateInfo(
                action=pose_action,
                subaction_path=role_path,
                #pose_in_action_space=xr.Posef()
            )
        ) for role_path in role_paths],
    )


    n_paths = ctypes.c_uint32(0)
    result = enumerateViveTrackerPathsHTCX(instance, 0, byref(n_paths), None)
    if xr.check_result(result).is_exception():
        raise result
    vive_tracker_paths = (xr.ViveTrackerPathsHTCX * n_paths.value)(*([xr.ViveTrackerPathsHTCX()] * n_paths.value))
    # print(xr.Result(result), n_paths.value)
    result = enumerateViveTrackerPathsHTCX(instance, n_paths, byref(n_paths), vive_tracker_paths)
    if xr.check_result(result).is_exception():
        raise result


    calroot = calibration_file_data.get("TopLeft", xr.Posef()).position
    caltarget = calibration_file_data.get("TopRight", xr.Posef()).position
    calangle =  - (np.arctan2(caltarget.z - calroot.z, caltarget.x - calroot.x)-np.pi/4)  # Vector between root and target - 45 degrees to get the angle of the x axis
    c, s = np.cos(calangle), np.sin(calangle)
    calRot = np.array(((c, -s), (s, c)))
    calmaxValues = np.matmul(calRot, np.array([caltarget.x - calroot.x, (caltarget.z - calroot.z)])) 
    print(f"x axis angle: {np.rad2deg(calangle)}")

 

    # Loop over the render frames
    for frame_index, frame_state in enumerate(context.frame_loop()):

        if context.session_state == xr.SessionState.FOCUSED:
            active_action_set = xr.ActiveActionSet(
                action_set=context.default_action_set,
                subaction_path=xr.NULL_PATH,
            )
            xr.sync_actions(
                session=session,
                sync_info=xr.ActionsSyncInfo(
                    count_active_action_sets=1,
                    active_action_sets=ctypes.pointer(active_action_set),
                ),
            )

            n_paths = ctypes.c_uint32(0)
            result = enumerateViveTrackerPathsHTCX(instance, 0, byref(n_paths), None)
            if xr.check_result(result).is_exception():
                raise result
            vive_tracker_paths = (xr.ViveTrackerPathsHTCX * n_paths.value)(*([xr.ViveTrackerPathsHTCX()] * n_paths.value))
            # print(xr.Result(result), n_paths.value)
            result = enumerateViveTrackerPathsHTCX(instance, n_paths, byref(n_paths), vive_tracker_paths)
            if xr.check_result(result).is_exception():
                raise result
            # print(xr.Result(result), n_paths.value)
            # print(*vive_tracker_paths)

            found_tracker_count = 0
            for index, space in enumerate(tracker_action_spaces):
                space_location = xr.locate_space(
                    space=space,
                    base_space=context.space,
                    time=frame_state.predicted_display_time,
                )
                if space_location.location_flags & xr.SPACE_LOCATION_POSITION_VALID_BIT:
                    output = f"{role_strings[index]}: {space_location.pose.position},"
                    xr_quad = space_location.pose.orientation

                    transformed = np.array([space_location.pose.position.x - calroot.x, (space_location.pose.position.z - calroot.z)])
                    #rotate tranformed by calRot 
                    rotated = np.matmul(calRot, transformed)   


                    #print(output, output3 ,np.rad2deg(output2))
                    #print(transformed) #TODO das muss gedreht werdenum output 3
                    print(f"{role_strings[index]}: {rotated}, {rotated/calmaxValues}, calmaxValues: {calmaxValues}")



                    if calibrate[0] and role_strings[index] == "chest":
                        calibration_file_data["TopLeft"] = space_location.pose  #xr.Posef(orientation=Math.Pose.rotate_ccw_about_y_axis(0, [0.0,0.0,0.0]).orientation, position=space_location.pose.position)  
                        print(f'calibration_file_data["TopLeft"]: {calibration_file_data["TopLeft"]}')
                        calibrate[0] = False
                        recalculateTransforms(calibration_file_data, save=True)
                    if calibrate[1] and role_strings[index] == "chest":
                        calibration_file_data["TopRight"] = space_location.pose #xr.Posef(orientation=Math.Pose.rotate_ccw_about_y_axis(0, [0.0,0.0,0.0]).orientation, position=space_location.pose.position)
                        print(f'calibration_file_data["TopRight"]: {calibration_file_data["TopRight"]}')
                        calibrate[1] = False
                        recalculateTransforms(calibration_file_data, save=True)
                    if calibrate[2] and role_strings[index] == "chest":
                        calibration_file_data["BottomLeft"] = space_location.pose #xr.Posef(orientation=Math.Pose.rotate_ccw_about_y_axis(0, [0.0,0.0,0.0]).orientation, position=space_location.pose.position)
                        print(f'calibration_file_data["BottomLeft"]: {calibration_file_data["BottomLeft"]}')
                        calibrate[2] = False
                        recalculateTransforms(calibration_file_data, save=True)

                            
                    pos = space_location.pose.position
                    #Switch of y and z is intentional
                    output = json.dumps({"u":pos.x * 1000 , "w": pos.z * 1000, "v": pos.y * 1000})
                    mqtt_client.publish(MQTT_BASE_TOPIC+f"{index}/pos", payload=output, qos=0, retain=False)
                    found_tracker_count += 1
                    
            if found_tracker_count == 0:
                print("no trackers found")


        # Slow things down, especially since we are not rendering anything
        time.sleep(1.0)
        # Don't run forever
        if not keep_going:
            mqtt_client.loop_stop()
            break