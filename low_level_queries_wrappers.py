import rospy
from dill.pointers import parent

from neem_interface_python import pose_to_string
from neem_interface_python.neem_generation import *
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.world_concepts.world_object import Object

# contains the low level python query wrappers for the NEEM interface.
# these are supposed to be used within the high level queries which are describing the designators.

def query_log_navigate_action_description(action, parent_action):
    #rospy.loginfo(PC.GREEN + f"[NEEM] in Query log navigate action descriptions for action: {action.__dict__}" + PC.GREY)
    action_designator_description_ids = []
    # parent_action = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_HCOEBKZS" # backup
    robot_name = f"http://www.ease-crc.org/ont/CROMA.owl#" + World.robot.name
    #parent_action = action.parent_action
    for destination in action.target_locations:
        query = ""
        # create the subaction with Task
        sub_action = knowrob_client.once(f"add_subaction_with_task({atom(parent_action)}, 'http://www.ease-crc.org/ont/SOMA.owl#Navigating', SubAction).")['SubAction']
        # append subaction with Task to the rest
        query += (f"kb_project(("
              #f"[add_subaction_with_task('{parent_action}', 'http://www.ease-crc.org/ont/SOMA.owl#Navigating', SubAction)],"
              f"[new_iri(ActionDesigDesc, croma:'ActionDesignatorDescription'),"
              f"has_type(ActionDesigDesc, croma:'ActionDesignatorDescription'),"
              f"triple(ActionDesigDesc, dul:'describes', '{sub_action}')])).")
        action_desig_desc_id = knowrob_client.once(query)['ActionDesigDesc']
        # have to add_participant_with_role individually...
        knowrob_client.once(f"add_participant_with_role('{sub_action}', '{robot_name}', 'http://www.ease-crc.org/ont/SOMA.owl#AgentRole').")

        # go back to main query
        query = f"kb_project(("
        # log the location, differentiate between location designator description and pose
        if isinstance(destination, LocationDesignatorDescription): # todo test
              location_desig_individual = query_log_location_designator(destination) # assumes return value is an individual IRI corresponding to the logged designator
              if not isinstance(location_desig_individual, list):
                  location_desig_individual = [location_desig_individual]
              for k,e in enumerate(location_desig_individual):
                  query += (f"[new_iri(DestinationRole{k}, soma:'Destination'), has_type(DestinationRole{k}, soma:'Destination')],"
                            f"triple(DestinationRole{k}, dul:'classifies', {e}),"
                            f"triple('{action_desig_desc_id}', dul:'definesRole', DestinationRole{k}),")
        elif isinstance(destination, Pose):
              query += (f"[new_iri(PoseInst, soma:'6DPose'), has_type(PoseInst, soma:'6DPose')])),"
                        #f"triple(PoseInst, soma:'hasPositionData', {pose_to_string(destination)}),"
                        f"mem_tf_set(PoseInst, '{destination.frame}', {destination.position_as_list()}, {destination.orientation_as_list()}, {rospy.get_time()}). "
                        f"kb_project(([new_iri(DestinationParameter, croma:'DestinationPose'), has_type(DestinationParameter, croma:'DestinationPose')])),"
                        f"triple(DestinationParameter, dul:'classifies', PoseInst),"
                        f"triple('{action_desig_desc_id}', dul:'definesParameter', DestinationParameter),")

        # log the keep_joint_states parameter
        query += (f"kb_project(([new_iri(KeepJointStates, croma:'KeepJointStates'), has_type(KeepJointStates, croma:'KeepJointStates')])),"
                  f"triple('{action_desig_desc_id}', dul:'hasParameter', KeepJointStates),")
        if action.keep_joint_states:
            query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'true'),"
        else:
            query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'false'),"

        query += f"instance_of('{action_desig_desc_id}', Class)"
        query += f"))."

        bindings = {}
        bindings = knowrob_client.all_solutions(query)
        print(f"bindings: {bindings}")

        action_designator_description_ids.append(action_desig_desc_id) # with [0]? why is this a list?
        action.action_designator_description_id = action_designator_description_ids
    return action_designator_description_ids

# --- NAVIGATE ACTION RESOLVED ---
def query_log_navigate_action_resolved(action, parent_action):
    action_designator_resolved_ids = []
    # parent_action = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_HCOEBKZS" # backup
    robot_name = f"http://www.ease-crc.org/ont/CROMA.owl#" + World.robot.name
    #parent_action = action.parent_action
    for destination in action.target_locations:
        query = ""
        # create the subaction with Task
        sub_action = knowrob_client.once(f"add_subaction_with_task({atom(parent_action)}, 'http://www.ease-crc.org/ont/SOMA.owl#Navigating', SubAction).")['SubAction']
        # append subaction with Task to the rest
        query += (f"kb_project(("
              #f"[add_subaction_with_task('{parent_action}', 'http://www.ease-crc.org/ont/SOMA.owl#Navigating', SubAction)],"
              f"[new_iri(ActionDesigRes, croma:'ActionDesignatorResolved'),"
              f"has_type(ActionDesigRes, croma:'ActionDesignatorResolved'),"
              f"triple(ActionDesigRes, dul:'describes', '{sub_action}')])).")
        action_desig_res_id = knowrob_client.once(query)['ActionDesigRes']
        # have to add_participant_with_role individually...
        knowrob_client.once(f"add_participant_with_role('{sub_action}', '{robot_name}', 'http://www.ease-crc.org/ont/SOMA.owl#AgentRole').")

        # go back to main query
        query = f"kb_project(("
        # log the location, differentiate between location designator description and pose
        if isinstance(destination, LocationDesignatorDescription): # todo test
              location_desig_individual = query_log_location_designator(destination) # assumes return value is an individual IRI corresponding to the logged designator
              if not isinstance(location_desig_individual, list):
                  location_desig_individual = [location_desig_individual]
              for k,e in enumerate(location_desig_individual):
                  query += (f"[new_iri(DestinationRole{k}, soma:'Destination'), has_type(DestinationRole{k}, soma:'Destination')],"
                            f"triple(DestinationRole{k}, dul:'classifies', {e}),"
                            f"triple('{action_desig_res_id}', dul:'definesRole', DestinationRole{k}),")
        elif isinstance(destination, Pose):
              query += (f"[new_iri(PoseInst, soma:'6DPose'), has_type(PoseInst, soma:'6DPose')])),"
                        #f"triple(PoseInst, soma:'hasPositionData', {pose_to_string(destination)}),"
                        f"mem_tf_set(PoseInst, '{destination.frame}', {destination.position_as_list()}, {destination.orientation_as_list()}, {rospy.get_time()}). "
                        f"kb_project(([new_iri(DestinationParameter, croma:'DestinationPose'), has_type(DestinationParameter, croma:'DestinationPose')])),"
                        f"triple(DestinationParameter, dul:'classifies', PoseInst),"
                        f"triple('{action_desig_res_id}', dul:'definesParameter', DestinationParameter),")

        # log the keep_joint_states parameter
        query += (f"kb_project(([new_iri(KeepJointStates, croma:'KeepJointStates'), has_type(KeepJointStates, croma:'KeepJointStates')])),"
                  f"triple('{action_desig_res_id}', dul:'hasParameter', KeepJointStates),")
        if action.keep_joint_states:
            query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'true'),"
        else:
            query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'false'),"

        query += f"instance_of('{action_desig_res_id}', Class)"
        query += f"))."

        bindings = {}
        bindings = knowrob_client.all_solutions(query)
        print(f"bindings: {bindings}")

        action_designator_resolved_ids.append(action_desig_res_id) # with [0]? why is this a list?
        action.action_designator_description_id = action_designator_resolved_ids
    return action_designator_resolved_ids

# --- NAVIGATE ACTION PERFORMED ---
def query_log_navigate_action_performed(action, parent_action):
    action_designator_performed_ids = []
    # parent_action = "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_HCOEBKZS" # backup
    robot_name = f"http://www.ease-crc.org/ont/CROMA.owl#" + World.robot.name
    #parent_action = action.parent_action
    destination = action.target_location
    query = ""
    # create the subaction with Task
    sub_action = knowrob_client.once(f"add_subaction_with_task({atom(parent_action)}, 'http://www.ease-crc.org/ont/SOMA.owl#Navigating', SubAction).")['SubAction']
    # append subaction with Task to the rest
    query += (f"kb_project(("
            #f"[add_subaction_with_task('{parent_action}', 'http://www.ease-crc.org/ont/SOMA.owl#Navigating', SubAction)],"
            f"[new_iri(ActionDesigPer, croma:'ActionDesignatorPerformed'),"
            f"has_type(ActionDesigPer, croma:'ActionDesignatorPerformed'),"
            f"triple(ActionDesigPer, dul:'describes', '{sub_action}')])).")
    action_desig_performed_id = knowrob_client.once(query)['ActionDesigPer']
    # have to add_participant_with_role individually...
    knowrob_client.once(f"add_participant_with_role('{sub_action}', '{robot_name}', 'http://www.ease-crc.org/ont/SOMA.owl#AgentRole').")
    # add source pose
    query = f"kb_project(("
    if isinstance(action.robot_position, Pose):
          query += (f"[new_iri(PoseInst, soma:'6DPose'), has_type(PoseInst, soma:'6DPose')])),"
                    #f"triple(PoseInst, soma:'hasPositionData', {pose_to_string(destination)}),"
                    f"mem_tf_set(PoseInst, '{action.robot_position.frame}', {action.robot_position.position_as_list()}, {action.robot_position.orientation_as_list()}, {rospy.get_time()}). "
                    f"kb_project(([new_iri(SourceParameter, croma:'SourcePose'), has_type(SourceParameter, croma:'SourcePose')])),"
                    f"triple(SourceParameter, dul:'classifies', PoseInst),"
                    f"triple('{action_desig_performed_id}', dul:'definesParameter', DestinationParameter),")

    # add Destination pose
    query = f"kb_project(("
    if isinstance(destination, Pose):
          query += (f"[new_iri(PoseInst, soma:'6DPose'), has_type(PoseInst, soma:'6DPose')])),"
                   #f"triple(PoseInst, soma:'hasPositionData', {pose_to_string(destination)}),"
                    f"mem_tf_set(PoseInst, '{destination.frame}', {destination.position_as_list()}, {destination.orientation_as_list()}, {rospy.get_time()}). "
                    f"kb_project(([new_iri(DestinationParameter, croma:'DestinationPose'), has_type(DestinationParameter, croma:'DestinationPose')])),"
                    f"triple(DestinationParameter, dul:'classifies', PoseInst),"
                    f"triple('{action_desig_performed_id}', dul:'definesParameter', DestinationParameter),")

        # log the keep_joint_states parameter
    query += (f"kb_project(([new_iri(KeepJointStates, croma:'KeepJointStates'), has_type(KeepJointStates, croma:'KeepJointStates')])),"
              f"triple('{action_desig_performed_id}', dul:'hasParameter', KeepJointStates),")
    if action.keep_joint_states:
        query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'true'),"
    else:
        query += f"triple(KeepJointStates, dul:'hasParameterDataValue', 'false'),"
    # log torso height
    query += (f"kb_project(([new_iri(TorsoHeight, croma:'TorsoHeight'), has_type(TorsoHeight, croma:'TorsoHeight')])),"
                f"triple('{action_desig_performed_id}', dul:'hasParameter', TorsoHeight),"
                f"triple(TorsoHeight, dul:'hasParameterDataValue', '{action.robot_torso_height}'),")

    query += f"instance_of('{action_desig_performed_id}', Class)"
    query += f"))."

    bindings = {}
    bindings = knowrob_client.all_solutions(query)
    print(f"bindings: {bindings}")

    action_designator_performed_ids.append(action_desig_performed_id) # with [0]? why is this a list?
    action.action_designator_description_id = action_designator_performed_ids
    return action_designator_performed_ids

# --- LOCATION DESIGNATOR ---
def query_log_location_designator(location_designator: LocationDesignatorDescription):
    return ""

