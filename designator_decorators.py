import rospy

from neem_interface_python.neem_generation import *
from neem_interface_python.py_to_prolog_interface import init_neem_interface
from pycram.designators.action_designator import *
import functools


"""This file contains all the decorators which wrap the action designators for logging """

# ---------------- Automation of calling the NEEM interface ----------------
# Flag to track initialization state
initialized = False

# todo: this might move somewhere else or only contain the mapping of the decorators to classes
def init_neem():
    # connect the Designator classes to their respective decorators
    #on_create_navigate_action(NavigateAction)
    #on_create_navigate_action_performable(NavigateActionPerformable)
    action_logger(NavigateAction)
    action_logger(NavigateActionPerformable)
    # todo: does it make sense to have this here or maybe have it separated?
    # init everything else
    # global initialized
    # if not initialized:
    #     rospy.loginfo(PC.GREEN + "[NEEM] Initializing connection..." + PC.GREY)
    #     # Your initialization logic here, e.g., establishing connection
    #     init_neem_interface()
    #     start_episode()
    #     initialized = True
    #     create_robot("Raphael", "PR2")
    #     #neem_class_decorator(NavigateAction)
    #     #neem_class_decorator(NavigateActionPerformable)
    #     rospy.loginfo(PC.GREEN + "[NEEM] Connection established." + PC.GREY)




# --- Generic Logging Decorator ---

def action_logger(cls):
    """Generic decorator for logging different types of Actions."""
    original_init = cls.__init__

    def init_logging(self, *args, **kwargs):
        """Logs object creation and calls original __init__."""
        rospy.loginfo(PC.GREEN + f"[NEEM] Logging {self.__class__.__name__} instance" + PC.GREY)

        # Call class-specific logging function if available
        log_action_creation(self)

        # Call the original __init__ method
        original_init(self, *args, **kwargs)

        # Wrap class-specific methods dynamically **after initialization**
        wrap_methods(self)

    # 🔹 General Action Logging
    def log_action_creation(action):
        """Logs when an Action is created."""
        rospy.loginfo(PC.YELLOW + "[NEEM] {action} created: {action}" + PC.GREY)
        # differentiate between the different types of actions
        class_method_log_map = {
            "NavigateAction": log_navigate_action_creation,
            "PerceptionAction": log_perception_action_creation,
            "TransportingAction": log_transporting_action_creation
        }
        return action

    def wrap_methods(self):
        """Wraps resolve() and perform() for logging, based on the action class."""
        rospy.loginfo(PC.YELLOW + "In wrap methods" + PC.GREY)
        class_method_log_map = {
            "NavigateAction": {"resolve": log_navigation_action_resolved},
            "NavigateActionPerformable": {"perform": log_navigation_action_performed},
            "PerceptionAction": {"resolve": log_perception_action_resolved},
            "PerceptionActionPerformable": {"perform": log_perception_action_performed},
            "TransportingAction": {"resolve": log_transporting_action_resolved},
            "TransportingActionPerformable": {"perform": log_transporting_action_performed},
        }

        class_name = self.__class__.__name__
        if class_name not in class_method_log_map:
            return  # If the class is not in the map, do nothing

        for method_name, log_function in class_method_log_map[class_name].items():
            if hasattr(self, method_name):
                original_method = getattr(self, method_name)

                if callable(original_method) and not getattr(original_method, "_is_wrapped", False):
                    @functools.wraps(original_method)
                    def wrapped_method(*args, **kwargs):
                        logging.info(f"[{class_name}] - Before {method_name}: {self}")
                        #log_function(self)  # Call the class-specific log function
                        result = original_method(*args, **kwargs)
                        logging.info(f"[{class_name}] - After {method_name}: {self}")
                        log_function(self) # log after the method execution since it might change the object
                        return result

                    wrapped_method._is_wrapped = True  # Prevent double wrapping
                    setattr(self, method_name, wrapped_method)  # ✅ Use `setattr()` to replace the method

    cls.__init__ = init_logging  # Override __init__
    return cls

# --- Creation Action Logging ---
def log_navigate_action_creation(action):
    rospy.loginfo(PC.GREEN + f"[NEEM] NavigationAction created: {action}" + PC.GREY)
    return action

def log_perception_action_creation(action):
    rospy.loginfo(PC.GREEN + f"[NEEM] PerceptionAction created: {action}" + PC.GREY)
    return action

def log_transporting_action_creation(action):
    rospy.loginfo(PC.GREEN + f"[NEEM] TransportingAction created: {action}" + PC.GREY)
    return action


# --- Resolution Action Logging ---
# 🔹 NavigationAction Logging
def log_navigation_action_resolved(action):
    rospy.loginfo(PC.GREEN + f"[NEEM] NavigationAction resolved: {action}" + PC.GREY)
    return action

# 🔹 PerceptionAction Logging
def log_perception_action_resolved(action):
    rospy.loginfo(PC.YELLOW + "[NEEM] PerceptionAction resolved: {action}" + PC.GREY)
    return action

# 🔹 TransportingAction Logging
def log_transporting_action_resolved(action):
    rospy.loginfo(f"[NEEM] TransportingAction resolved: {action}")
    return action



# --- Perform Action Logging
def log_navigation_action_performed(action):
    rospy.loginfo(PC.GREEN + f"[NEEM] NavigationAction performed: {action}" + PC.GREY)
    return action

def log_perception_action_performed(action):
    rospy.loginfo(PC.YELLOW + "[NEEM] PerceptionAction performed: {action}" + PC.GREY)
    return action

def log_transporting_action_performed(action):
    rospy.loginfo(f"[NEEM] TransportingAction performed: {action}")
    return action
