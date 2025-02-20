import rospy
from dill.pointers import parent

from neem_interface_python import pose_to_string
from neem_interface_python.neem_generation import *
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.world_concepts.world_object import Object

# contains the low level python query wrappers for the NEEM interface.
# these are supposed to be used within the high level queries which are describing the designators.
# to add to croma:
# KeepJointStates: subclass of dul:Parameter
#     annotations: comment containing definition from pycram



# TODO: fix
# TODO: assert robot
# robot ID can be whatever, should look URI-like, so could be pycram.com#PR2_0
# triple(My_Robot_ID, rdf:'type', soma:'Robot')
# rdfs:'subclassOf'

#log Navigation actions with one pose essentially, have multiple of them in sequence if necessary.

#case 1: destination is location designator description
#              BLA = query_log_location_designator(action.targetLocations[k]) # assumes return value is an individual IRI corresponding to the logged designator
#              query += (f"[new_iri(DestinationRole, soma:'Destination'), has_type(DestinationRole, soma:'Destination')],"
#                        f"triple(DestinationRole, dul:'classifies', {BLA}),"
#                        f"triple(ActionDesigDesc, dul:'definesRole', DestinationRole)")
#case 2: destination is a pose
#              query += (f"[new_iri(PoseInst, soma:'6DPose'), has_type(PoseInst, soma:'6DPose')],"
#                        f"triple(PoseInst, soma:hasPositionData, {poseToString(action.targetLocations[k])})"
#                        f"[new_iri(DestinationParameter, soma:'Setpoint'), has_type(PoseInst, soma:'Setpoint')],"
#                        f"triple(DestinationParameter, dul:'classifies', PoseInst)"
#                        f"triple(ActionDesigDesc, dul:'definesParameter', DestinationParameter),")





def query_log_navigate_action_description(action):
    #rospy.loginfo(PC.GREEN + f"[NEEM] in Query log navigate action descriptions for action: {action.__dict__}" + PC.GREY)
    action_designator_description_ids = []
    # parent_action = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_HCOEBKZS" # backup
    robot_name = f"http://www.ease-crc.org/ont/CROMA.owl#" + World.robot.name
    #parent_action = action.parent_action
    for destination in action.target_locations:
        query = ""
        # create the subaction with Task
        query += (f"kb_project(("
              f"[add_subaction_with_task('{parent_action}', soma:'Navigating', SubAction),"
              f"new_iri(ActionDesigDesc, croma:'ActionDesignatorDescription'),"
              f"has_type(ActionDesigDesc, croma:'ActionDesignatorDescription'),"
              f"triple(ActionDesigDesc, dul:'describes', SubAction)],"
              f"add_participant_with_role(SubAction, {atom(robot_name)}, soma:'AgentRole'),")

        # log the location, differentiate between location designator description and pose
        if isinstance(destination, LocationDesignatorDescription):
              location_desig_individual = query_log_location_designator(destination) # assumes return value is an individual IRI corresponding to the logged designator
              if not isinstance(location_desig_individual, list):
                  location_desig_individual = [location_desig_individual]
              for k,e in enumerate(location_desig_individual):
                  query += (f"[new_iri(DestinationRole{k}, soma:'Destination'), has_type(DestinationRole{k}, soma:'Destination')],"
                            f"triple(DestinationRole{k}, dul:'classifies', {e}),"
                            f"triple(ActionDesigDesc, dul:'definesRole', DestinationRole{k}),")
        elif isinstance(destination, Pose):
              query += (f"[new_iri(PoseInst, soma:'6DPose'), has_type(PoseInst, soma:'6DPose')],"
                        f"triple(PoseInst, soma:hasPositionData, {pose_to_string(destination)}),"
                        f"[new_iri(DestinationParameter, soma:'Setpoint'), has_type(PoseInst, soma:'Setpoint')],"
                        f"triple(DestinationParameter, dul:'classifies', PoseInst),"
                        f"triple(ActionDesigDesc, dul:'definesParameter', DestinationParameter),")

        # log the keep_joint_states parameter
        query += (f"[new_iri(KeepJointStates, croma:'KeepJointStates'), has_type(KeepJointStates, croma:'KeepJointStates')],"
                  f"triple(ActionDesigDesc, dul:'hasParameter', KeepJointStates),")
        if action.keep_joint_states:
            query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'true'),"
        else:
            query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'false'),"

        query += f"instance_of(ActionDesigDesc, Class)"
        query += f"))."

        bindings = knowrob_client.once(query)
        print(f"bindings: {bindings}")

        action_designator_description_ids.append(bindings["ActionDesigDesc"]) # with [0]?
        action.action_designator_description_id = action_designator_description_ids
    return action_designator_description_ids

# --- LOCATION DESIGNATOR ---
def query_log_location_designator(location_designator: LocationDesignatorDescription):
    return ""

