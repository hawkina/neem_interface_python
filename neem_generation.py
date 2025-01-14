import functools
import inspect
import rospy
from neem_interface_python.py_to_prolog_interface import *
from neem_interface_python.utils import PC, atom
# from demos.pycram_gpsr_demo import utils
from pycram.designators import *
from pycram.designators.action_designator import *
from pycram.designators.action_designator import ActionAbstract
from pycram.designators.object_designator import *
from neem_interface_python import py_to_prolog_interface
import pycram.tasktree as tt # needed to access the performed action

"""
This file contains the logic to generate NEEM entries for the actions performed in the CRAM plan.
This means that it contains the hook which is called when an action designator is created, resolved or performed,
which in turn sends the action designator data to KnowRob in order to generate a NEEM.
It also contains the variables which need to be set in advance, describing the urdf files, owl files, etc.
"""



task_type = "Serve Breakfast"
# todo change from apartment to robocuop arena
env_owl = "package://iai_apartment/owl/iai-apartment.owl"
env_owl_ind_name = "http://knowrob.org/kb/iai-apartment.owl#apartment_root"  # ind = individual
env_urdf = "package://iai_apartment/urdf/apartment.urdf"
env_urdf_prefix = "iai_apartment/"
# agent_owl = "package://knowrob/owl/robots/hsrb.owl"
agent_owl = "package://knowrob/owl/robots/PR2.owl"
# agent_owl_ind_name = "http://knowrob.org/kb/hsrb.owl#hsrb_robot1"
agent_owl_ind_name = "http://knowrob.org/kb/PR2.owl#PR2_0"
# agent_urdf = "package://knowrob/urdf/hsrb.urdf"
agent_urdf = "package://knowrob/urdf/pr2.urdf"
neem_output_path = "/home/hawkin/ros_ws/neems_library/serve_breakfast_neems/"
start_time = None
root_action = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_ZBRTKSFH"  # remove later

#### temp. remove later ####
transporting = "http://www.ease-crc.org/ont/SOMA.owl#Transporting"
# toya = "http://www.ease-crc.org/ont/SUTURO.owl#ToyotaHSR_IAMTOYA"
robot = "http://www.ease-crc.org/ont/SUTURO.owl#PR2_0"

# internal?
def start_episode():
    global root_action

    res = py_to_prolog_interface.start_episode(task_type, env_owl, env_owl_ind_name, env_urdf,
                        env_urdf_prefix, agent_owl,
                        agent_owl_ind_name, agent_urdf)
    root_action = res
    return root_action


def stop_and_dump_episode():
    rospy.loginfo(PC.GREEN + f"[NEEM] Stopping episode... Path: {neem_output_path}" + PC.GREY)
    stop_episode(neem_output_path)
    return neem_output_path


# def add_subaction_with_task(parent_action=root_action, action_type=transporting):  # TODO remove transporting
#     res = add_subaction_with_task(parent_action, action_type)
#     return res


# result: the iri of the subaction. e.g. http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_PBKJOHZG'

# ---------------- Automation of calling the NEEM interface ----------------
# Flag to track initialization state
initialized = False
parent_action = root_action
current_action = None


# Define the initialization function that establishes the connection
def initialize_neem():  # done
    global initialized
    if not initialized:
        rospy.loginfo(PC.GREEN + "[NEEM] Initializing connection..." + PC.GREY)
        # Your initialization logic here, e.g., establishing connection
        init_neem_interface()
        start_episode()
        initialized = True
        rospy.loginfo(PC.GREEN + "[NEEM] Connection established." + PC.GREY)


# Class-level decorator to handle object initialization
def neem_class_decorator(cls):
    original_init = cls.__init__

    # --- CREATES NEEM AT ACTION CREATION TIME ---
    @functools.wraps(original_init)
    def new_init(self, *args, **kwargs):
        global parent_action, current_action
        # Ensures initialization is done once when an object is created
        initialize_neem()
        original_init(self, *args, **kwargs)  # Call the original __init__
        rospy.loginfo(PC.PINK + f"[NEEM] Initializing object of class {cls.__name__}" + PC.GREY)
        current_action_description_instance = None
        if isinstance(self, NavigateAction):
            #rospy.loginfo(PC.RED + f"[NEEM] NavigateActionPerformable: {self.__dict__}" + PC.GREY)

            if isinstance(self, NavigateAction):
                rospy.loginfo(PC.GREEN + "[NEEM] NavigateAction detected" + PC.GREY)
                current_action, current_action_description_instance = add_subaction_with_task(parent_action=parent_action,
                                                                                         task_type="soma:Navigating")
                add_participant_with_role(current_action, robot, "soma:AgentRole")
                # todo add goal which is the location designator
                # self.target_locations
                # create an instance of a location
                loc_inst = make_instance_of("soma:Location")
                # connect location instance to action as goal
                triple(current_action, "soma:hasGoal", loc_inst)  # add a pose

                # --- DESCRIPTION START ---
                # process location designator within the action designator
                for attr_name, attr_value in self.__dict__.items():
                    # this is location designator specific todo test if only for my locdesig or generally for all of them
                    rospy.loginfo(PC.YELLOW + f"[NEEM] processing attribute: {attr_name} + {attr_value} " + PC.GREY)
                    if isinstance(attr_value, Location): # account for a Location Designator being given as a position
                        rospy.loginfo(PC.YELLOW + f"[NEEM] Found location designator: {attr_name} + {attr_value} " + PC.GREY)

                        # take care of ObjectDesignatorDescriptions within the LocationDesignatorDescription
                        param_list = {}
                        for item_name in attr_value.kwargs:
                            item = attr_value.kwargs.get(item_name)
                            rospy.loginfo(PC.GREEN + f"[NEEM] Processing: {item_name} value: {item}" + PC.GREY)
                            if isinstance(item, ObjectDesignatorDescription):
                                rospy.loginfo(PC.GREEN + f"[NEEM] Processing ObjectDesignatorDescription Parameter within LocationDesignatorDescription" + PC.GREY)
                                # add object designator to NEEM
                                obj_desig_design = add_object_designator_description(item)
                                param_list[item_name] = obj_desig_design
                                rospy.loginfo(PC.GREEN + f"[NEEM] Done Processing ObjectDesignatorDescription Parameter within LocationDesignatorDescription: {obj_desig_design}" + PC.GREY)
                            else:
                                rospy.loginfo(PC.YELLOW + f"[NEEM] Not an instance of ObjectDesignatorDescription: {item}. Moving On." + PC.GREY)
                        # add all the parameters to the location designator description instance
                        rospy.loginfo(
                            PC.GREEN + f"[NEEM] Adding all parameters to LocationDesignatorDescription instance: {param_list}" + PC.GREY)
                        loc_desig_desc_instance = add_location_designator_description(param_list)
                        # add location desig description instance to the action
                        triple(current_action_description_instance, "soma:hasGoal", loc_desig_desc_instance)
                        triple(current_action_description_instance, "soma:hasLocation", loc_desig_desc_instance)

                        rospy.loginfo(
                            PC.GREEN + f"[NEEM] Done Processing LocationDesignatorDescription Parameter within LocationDesignatorDescription: {loc_desig_desc_instance}" + PC.GREY)
                        # --- DESCRIPTION END ---
                        # --- RESOLVE START ---
                        # grounding of values, e.g. resolution of location designator
                        resolved_location_designator = attr_value.ground()  # Location.ground()
                        resolved_desig = add_resolved_location_designator(resolved_location_designator,
                                                                          loc_desig_desc_instance)
                        rospy.loginfo(PC.GREEN + f"[NEEM] Done processing LocationDesignator: {resolved_desig}" + PC.GREY)

                    if isinstance(attr_value, list) and attr_name.__contains__("target_location"):
                        rospy.loginfo(PC.GREEN + f"[NEEM] list of poses detected" + PC.GREY)
                        # log Navigation Designator if only the pose is given instead of a Location Designator
                        # make location instance
                        add_location_with_poses_list(current_action_description_instance, attr_value)
                        # --- DESCRIPTION END ---
                        # --- RESOLVE START ---
                # fallback if action type could not be matched
                if current_action_description_instance is None:
                    current_action, current_action_description_instance = add_subaction_with_task(parent_action=parent_action,
                                                                                                  task_type="soma:Action")
                self.action_designator_description_id = current_action_description_instance
                # --- RESOLVE END ---

                # Wrap resolve and perform methods of ActionDesignator to log them
                self.resolve = log_method(self.resolve, self, 'resolve')
                self.perform = log_method(self.perform, self, 'perform')
        else:
            rospy.logerr(f"[NEEM] not an instance of ActionDesignator: {self}" + PC.GREY)

    cls.__init__ = new_init
    return cls


# --- Decorator to generate NEEM entries for a method -> Resolve and Perform---
# This decorator logs calls to the resolve and perform methods
def log_method(method, action_designator, method_name):
    @functools.wraps(method)
    def wrapper(*args, **kwargs):
        rospy.loginfo(PC.PINK + f"[NEEM] In Log_method wrapper {method_name}() on {action_designator}" + PC.GREY)

        if method_name == 'resolve':
            # todo add type specific handling
            rospy.loginfo(PC.PINK + f"[NEEM] Starting {method_name} on {action_designator.__dict__}" + PC.GREY)
            # connect to parent e.g. design e.g. ActionDescription
            action_designator_description_id = action_designator.action_designator_description_id
            action_id = create_action_id(action_designator)
            triple(action_designator_description_id, f"dul:expresses", action_id)
            resolved_action = method(*args, **kwargs)  # actually resolve the designator
            resolved_action.action_id = action_id
            resolved_action.action_designator_description_id = action_designator_description_id
            target_location = make_instance_of("soma:Location")
            pose_array = [resolved_action.target_location.header.frame_id] + resolved_action.target_location.to_list()
            add_pose_to_instance(target_location, pose_array)
            # todo add robot_type (ENUM)
            # todo add robot_torso_height
            rospy.loginfo(PC.PINK + f"[NEEM] Resolution result: {resolved_action.__dict__}" + PC.GREY)
            result = resolved_action

        elif method_name == 'perform':
            # Log start time for perform
            action_begin(current_action)  # double check if needed
            rospy.loginfo(PC.PINK + f"[NEEM] Starting {method_name} on {action_designator}" + PC.GREY)

            # Call the original method and log the result
            result = method(*args, **kwargs)
            performed_desig = result
            all_args = inspect.getcallargs(method, *args, **kwargs)
            rospy.loginfo(PC.PINK + f"[NEEM] {method_name}() result: {result}" + PC.GREY)

            # Log end time for perform
            action_end(current_action)  # double check if need
            rospy.loginfo(
                PC.PINK + f"[NEEM] {method_name}() completed on {action_designator}, result: {result}" + PC.GREY)
        else:
            result = method(*args, **kwargs)
            rospy.loginfo(PC.PINK + f"[NEEM] Neither perform or resolve: {method_name}() result: {result}" + PC.GREY)
        return result

    return wrapper


# Function to dynamically apply decorators to your ActionDesignator class
# potentionally removable
def enable_neem_generation():
    class NeemClass:
        def __init__(self, type, **kwargs):
            self.type = type
            rospy.loginfo(PC.YELLOW + f"[NEEM] ActionDesignator initialized with type {self.type}" + PC.GREY)  # ?

        def resolve(self):
            # Logic for resolving designator
            rospy.loginfo(PC.YELLOW + f"[NEEM] Resolving designator: {self}" + PC.GREY)
            log_method(self.resolve, self, 'resolve')
            return "--- resolved_result ---"

        def perform(self):
            # Logic for performing action
            rospy.loginfo(PC.YELLOW + f"[NEEM] Performing action: {self}" + PC.GREY)
            log_method(self.perform, self, 'perform')
            return "--- action_performed ---"

    # Apply class-level decorator
    NeemClass = neem_class_decorator(NavigateAction)
    # Apply method-level decorator to specific methods
    # Apply method-level decorator to specific methods
    # ActionDesignator.resolve = generate_neem(ActionDesignator.resolve)
    # ActionDesignator.perform = generate_neem(ActionDesignator.perform)
    # TODO this should be dynamic
    # NavigateActionPerformable.perform = generate_neem(NavigateActionPerformable.perform)
    # DetectActionPerformable.perform = generate_neem(DetectActionPerformable.perform)

    return NeemClass


def reset_neem():
    global initialized, parent_action, current_action
    initialized = False
    parent_action = root_action
    current_action = None


# --- NEEM queries ----
# def get_longest_event():
#     result = prolog_client.all_solutions(f"findall([Duration, Evt],"
#                                                f"(event_interval(Evt, Begin, End),"
#                                                f"number(End),"
#                                                f"Duration is End - Begin),"
#                                                f"Durations),"
#                                                f"max_member([MaxDuration, LongestEvt], Durations).")
#     return result
