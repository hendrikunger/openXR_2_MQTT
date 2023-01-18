
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

mqtt_broker = "broker.hivemq.com"
#mqtt_broker = "10.54.129.47"
MQTT_BASE_TOPIC ="EDF/BP/machines/"


keep_going = True
calibrate = False

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


#Detect Keys (for exit)
def on_press(key):
    global keep_going
    global calibrate
    try:
        if key.char == "c":
            print("Calibrating Position")
            calibrate = True
            return
        if key.char == "l":
            path = os.path.realpath(os.path.dirname(__file__))
            with open( path+"/cal.p", "rb" ) as f:
                cal = pickle.load(f)
                print(cal, type(cal))
            return

        print('alphanumeric key {0} pressed'.format(key.char))

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
    client.subscribe("EDF/BP/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

mqtt_client = mqtt.Client()


mqtt_client = mqtt.Client(client_id="Tracker")
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker, 1883)
mqtt_client.loop_start()

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
    # Create action spaces for locating trackers in each role
    tracker_action_spaces = (xr.Space * len(role_paths))(
        *[xr.create_action_space(
            session=session,
            create_info=xr.ActionSpaceCreateInfo(
                action=pose_action,
                subaction_path=role_path,
                pose_in_action_space= xr.Posef(), ### TODO Position of Actions Space set to calibration
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
    print(xr.Result(result), n_paths.value)
    # print(*vive_tracker_paths)

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
                    if calibrate and index == 1:
                        path = os.path.realpath(os.path.dirname(__file__))
                        with open( path+"/cal.p", "wb" ) as f:
                            pickle.dump(space_location.pose,  f)
                        calibrate = False
                        print("Calibration finished. Please Restart")

                    pos = space_location.pose.position
                    #Switch of y and z is intentional
                    output = json.dumps({"x":pos.x * 1000 , "y": pos.z * 1000, "z": pos.y * 1000})
                    index="0Eqz09Em50ZgMSsOvwCNok"
                    mqtt_client.publish(MQTT_BASE_TOPIC+f"{index}/pos", payload=output, qos=0, retain=False)
                    found_tracker_count += 1
            if found_tracker_count == 0:
                print("no trackers found")

        # Slow things down, especially since we are not rendering anything
        time.sleep(0.5)
        # Don't run forever
        if not keep_going:
            mqtt_client.loop_stop()
            break
