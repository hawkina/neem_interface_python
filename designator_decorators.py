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


# 🔹 General Action Logging
def log_action_creation(action):
    """Logs when an Action is created."""
    rospy.loginfo(PC.YELLOW + "[LOG] {action} created: {action}" + PC.GREY)
    return action


# 🔹 NavigationAction Logging
def log_navigation_action_resolved(action):
    rospy.loginfo(PC.GREEN + f"[LOG] NavigationAction resolved: {action}" + PC.GREY)
    return action


def log_navigation_action_performed(action):
    rospy.loginfo(PC.GREEN + f"[LOG] NavigationAction performed: {action}" + PC.GREY)
    return action


# 🔹 PerceptionAction Logging
def log_perception_action_resolved(action):
    rospy.loginfo(PC.YELLOW + "[LOG] PerceptionAction resolved: {action}" + PC.GREY)
    return action


def log_perception_action_performed(action):
    rospy.loginfo(PC.YELLOW + "[LOG] PerceptionAction performed: {action}" + PC.GREY)
    return action


# 🔹 TransportingAction Logging
def log_transporting_action_resolved(action):
    rospy.loginfo(f"[LOG] TransportingAction resolved: {action}")
    return action


def log_transporting_action_performed(action):
    rospy.loginfo(f"[LOG] TransportingAction performed: {action}")
    return action
# def on_create_navigate_action(cls):
#     """Class decorator to log NavigateAction creation and dynamically wrap resolve()."""
#     original_init = cls.__init__  # Save the original __init__ method
#
#     def init_logging(self, *args, **kwargs):
#         rospy.loginfo(PC.GREEN + f"[NEEM] Logging NA. {self.__class__.__name__} " + PC.GREY)
#         original_init(self, *args, **kwargs)  # Call the original __init__ method
#
#         # Ensure we only wrap resolve() once
#         if hasattr(self.__class__, "resolve") and not getattr(self.resolve, "_is_wrapped", False):
#             original_resolve = self.resolve
#
#             @functools.wraps(original_resolve)
#             def wrapped_resolve(*args, **kwargs):
#                 rospy.loginfo(f"NA - Before resolve: {self}")
#                 result = original_resolve(*args, **kwargs)
#                 rospy.loginfo(f"NA - After resolve: {self}")
#                 return result
#
#             wrapped_resolve._is_wrapped = True  # Mark as wrapped to prevent re-wrapping
#             setattr(self, "resolve", wrapped_resolve)
#
#     cls.__init__ = init_logging  # Replace the __init__ method with the new one
#     return cls
# # ---
#
# @with_tree
# def on_create_navigate_action_performable(cls):
#     # log the NavigateAction on creation (before any resolution)
#     original_init = cls.__init__  # Save the original __init__ method
#
#     def init_logging(self, *args, **kwargs):
#         rospy.loginfo(PC.GREEN + f"[NEEM] Logging NAP: {self.__class__.__name__} " + PC.GREY )
#         original_init(self, *args, **kwargs)  # Call the original __init__ method
#
#         # Wrap the perform method dynamically
#         if not getattr(self.perform, "_is_wrapped", False):
#             original_perform = self.perform
#
#             @functools.wraps(original_perform)
#             def wrapped_perform(*args, **kwargs):
#                 rospy.loginfo(f"NAP - Before perform: {self}")
#                 result = original_perform(*args, **kwargs)
#                 rospy.loginfo(f"NA - After perform: {self}")
#                 return result
#
#             wrapped_perform._is_wrapped = True  # Mark the method as wrapped
#             setattr(self, "perform", wrapped_perform)
#
#     cls.__init__ = init_logging  # Replace the __init__ method with the new one
#
#     # def log_method(method, action_designator, method_name):
#     #     @functools.wraps(method)
#     #     def wrapper(*args, **kwargs):
#     #         rospy.loginfo(PC.GREEN + "[NEEM] Logging NAP - now methods." + {cls.__name__} + PC.GREY )
#     #
#     #         if method_name == 'resolve':
#     #             rospy.loginfo(PC.GREEN + "[NEEM] Logging NAP - resolve." + {cls.__name__} + PC.GREY )
#     #         elif method_name == 'perform':
#     #             rospy.loginfo(PC.GREEN + "[NEEM] Logging NAP - perform." + {cls.__name__} + PC.GREY )
#     #         else:
#     #             rospy.loginfo(PC.GREEN + "[NEEM] Logging NAP - No matching method found." + {cls.__name__} + PC.GREY )
#     #             return method(*args, **kwargs)
#
#     return cls