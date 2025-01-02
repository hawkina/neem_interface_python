import os
from typing import List, Tuple, Optional
import time

import rospy.rostime
from tqdm import tqdm

from neem_interface_python.rosprolog_client import Prolog
from neem_interface_python.utils import atom
# from rosprolog_client import Prolog, PrologException
from neem_interface_python.utils.utils import Datapoint, Pose

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

knowrob_client = Prolog()


def init_neem_interface():
    """
    Initializes the Neem interface. Please provide a Prolog client to connect to.
    """
    neem_interface_path = os.path.join(SCRIPT_DIR, "src", "neem-interface", "src", "neem-interface.pl")
    print(f"path: {neem_interface_path}")
    knowrob_client.once(f"ensure_loaded({atom(neem_interface_path)}).")

    ### NEEM Creation ###############################################################


def start_episode(task_type: str, env_owl: str, env_owl_ind_name: str, env_urdf: str, env_urdf_prefix: str,
                  agent_owl: str, agent_owl_ind_name: str, agent_urdf: str, start_time: float = None):
    """
    Start an episode and return the prolog atom for the corresponding action.
    E.g. res = 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_WBEKCRYQ'
    """
    # q = f"mem_episode_start(Action, {atom(task_type)}, {atom(env_owl)}, {atom(env_owl_ind_name)}, {atom(env_urdf)}," \
    #     f"{atom(agent_owl)}, {atom(agent_owl_ind_name)}, {atom(agent_urdf)}," \
    #     f"{start_time if start_time is not None else time.time()})"
    q = f"mem_episode_start(Action, {atom(env_owl)}, {atom(env_owl_ind_name)}, {atom(env_urdf)}," \
        f"{atom(env_urdf_prefix)},{atom(agent_owl)}, {atom(agent_owl_ind_name)}, {atom(agent_urdf)}" \
        f")."
    # f"{start_time if start_time is not None else time.time()})"

    res = knowrob_client.once(q)
    return res["Action"]


def stop_episode(neem_path: str, end_time: float = None):
    """
    End the current episode and save the NEEM to the given path
    """
    return knowrob_client.once(
        f"mem_episode_stop({atom(neem_path)}, {end_time if end_time is not None else time.time()})")


def add_subaction_with_task(parent_action, task_type="dul:'Task'", sub_action_type="dul:'Action'",
                            start_time: float = None, end_time: float = None) -> str:
    """
    Assert a subaction of a given type, and an associated task of a given type.
    """
    q = f"add_subaction_with_task({atom(parent_action)}, {atom(task_type)}, SubAction)"
    solution = knowrob_client.once(q)
    action_iri = solution["SubAction"]
    action_designator_design_iri = knowrob_client.once(
        f"kb_project([new_iri(ActionDesigDesc, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Action_Designator_Design'),"
        f"has_type(ActionDesigDesc, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Action_Designator_Design'),"
        f"triple(ActionDesigDesc, dul:'describes', {atom(action_iri)})]),"
        f"instance_of(ActionDesigDesc, Class).")

    if start_time is not None and end_time is not None:
        knowrob_client.once(f"kb_project(has_time_interval({atom(action_iri)}, {start_time}, {end_time}))")
    return action_iri, action_designator_design_iri["ActionDesigDesc"]


def belief_perceived_at(object_type, mesh, position, rotation):
    # add an object which has been perceived by perception to knowrob
    q = f"belief_perceived_at({atom(object_type)}, {atom(mesh)}, {atom(position)}, {atom(rotation)})"
    res = knowrob_client.once(q)
    # returns the whole object with the Knowledge ID
    return res


def create_action_id(self):
    res = knowrob_client.once(f"kb_project([new_iri(ActionDesig, soma_cram:'Action_Designator'),"
                                  f"has_type(ActionDesig, soma_cram:'Action_Designator')])")
    return res["ActionDesig"]


def add_participant_with_role(action: str, participant: str, role_type="dul:'Role'") -> None:
    """
    Assert that something was a participant with a given role in an action.
    Participant must already have been inserted into the knowledge base.
    """
    q = f"add_participant_with_role({atom(action)}, {atom(participant)}, {atom(role_type)})"
    knowrob_client.once(q)


def action_begin(current_action: str):
    q = f"mem_action_begin({atom(current_action)})"
    knowrob_client.once(q)


def action_end(current_action: str):
    q = f"mem_action_end({atom(current_action)})"
    knowrob_client.once(q)


def assert_tf_trajectory(points: List[Datapoint]):
    print(f"Inserting {len(points)} points")
    for point in tqdm(points):
        ee_pose_str = point.to_knowrob_string()
        knowrob_client.once(f"""
                time_scope({point.timestamp}, {point.timestamp}, QS),
                tf_set_pose({atom(point.frame)}, {ee_pose_str}, QS).
            """)


def assert_transition(agent_iri: str, object_iri: str, start_time: float, end_time: float) -> Tuple[
    str, str, str]:
    res = knowrob_client.once(f"""
            kb_project([
                new_iri(InitialScene, soma:'Scene'), is_individual(InitialScene), instance_of(InitialScene, soma:'Scene'),
                new_iri(InitialState, soma:'State'), is_state(InitialState),
                has_participant(InitialState, {atom(object_iri)}),
                has_participant(InitialState, {atom(agent_iri)}),
                holds(InitialScene, dul:'includesEvent', InitialState),
                has_time_interval(InitialState, {start_time}, {start_time}),

                new_iri(TerminalScene, soma:'Scene'), is_individual(TerminalScene), instance_of(TerminalScene, soma:'Scene'),
                new_iri(TerminalState, soma:'State'), is_state(TerminalState),
                has_participant(TerminalState, {atom(object_iri)}),
                has_participant(TerminalState, {atom(agent_iri)}),
                holds(TerminalScene, dul:'includesEvent', TerminalState),
                has_time_interval(TerminalState, {end_time}, {end_time}),

                new_iri(Transition, dul:'Transition'), is_individual(Transition), instance_of(Transition, soma:'StateTransition'),
                holds(Transition, soma:'hasInitialScene', InitialScene),
                holds(Transition, soma:'hasTerminalScene', TerminalScene)
            ]).
        """)
    transition_iri = res["Transition"]
    initial_state_iri = res["InitialState"]
    terminal_state_iri = res["TerminalState"]
    return transition_iri, initial_state_iri, terminal_state_iri


def assert_agent_with_effector(effector_iri: str, agent_type="dul:'PhysicalAgent'", agent_iri: str = None) -> str:
    if agent_iri is None:
        agent_iri = knowrob_client.once(f"""
                kb_project([
                    new_iri(Agent, dul:'Agent'), is_individual(Agent), instance_of(Agent, {atom(agent_type)})
                ]).""")["Agent"]
    knowrob_client.once(f"kb_project(has_end_link({atom(agent_iri)}, {atom(effector_iri)}))")
    return agent_iri


def assert_state(participant_iris: List[str], start_time: float = None, end_time: float = None,
                 state_class="soma:'State'", state_type="soma:'StateType'") -> str:
    state_iri = knowrob_client.once(f"""
            kb_project([
                new_iri(State, soma:'State'), is_individual(State), instance_of(State, {atom(state_class)}),
                new_iri(StateType, soma:'StateType'), is_individual(StateType), instance_of(StateType, {atom(state_type)}),
                holds(StateType, dul:'classifies',  State)
            ])
        """)["State"]
    if start_time is not None and end_time is not None:
        knowrob_client.once(f"kb_project(has_time_interval({atom(state_iri)}, {start_time}, {end_time}))")
    for iri in participant_iris:
        knowrob_client.once(f"kb_project(has_participant({atom(state_iri)}, {atom(iri)}))")
    return state_iri


def assert_situation(agent_iri: str, involved_objects: List[str], situation_type="dul:'Situation'") -> str:
    situation_iri = knowrob_client.once(f"""
            kb_project([
                new_iri(Situation, {atom(situation_type)}), is_individual(Situation), instance_of(Situation, {atom(situation_type)}),
                holds(Situation, dul:'includesAgent', {atom(agent_iri)})
            ])
        """)["Situation"]
    for obj_iri in involved_objects:
        knowrob_client.once(f"kb_project(holds({atom(situation_iri)}, dul:'includesObject', {atom(obj_iri)}))")
    return situation_iri


def assert_object_pose(obj_iri: str, obj_pose: Pose, start_time: float = None, end_time: float = None):
    if start_time is not None and end_time is not None:
        qs_query = f"time_scope({start_time}, {end_time}, QS)"
    elif start_time is not None and end_time is None:
        qs_query = f"time_scope({start_time}, {time.time()}, QS)"
    else:
        qs_query = f"time_scope({time.time()}, {time.time()}, QS)"
    knowrob_client.once(f"{qs_query}, tf_set_pose({atom(obj_iri)}, {obj_pose.to_knowrob_string()}, QS)")


### NEEM Parsing ###############################################################

def load_neem(neem_path: str):
    """
    Load a NEEM into the KnowRob knowledge base.
    """
    knowrob_client.once(f"remember({atom(neem_path)})")


# def get_all_actions(self) -> List[str]:
#     res = knowrob_client.all_solutions("is_action(Action)")
#     if len(res) > 0:
#         return list(set([dic["Action"] for dic in
#                          res]))  # Deduplicate: is_action(A) may yield the same action more than once
#     else:
#         raise NEEMError("Failed to find any actions")


def get_interval_for_action(action: str) -> Optional[Tuple[float, float]]:
    res = knowrob_client.once(f"event_interval({atom(action)}, Begin, End)")
    if res is None:
        return res
    return res["Begin"], res["End"]


def get_object_pose(obj: str, timestamp: float = None) -> Pose:
    if timestamp is None:
        query = f"mem_tf_get({atom(obj)}, Pose)"
    else:
        query = f"mem_tf_get({atom(obj)}, Pose, {timestamp})"
    return Pose.from_prolog(knowrob_client.once(query)["Pose"])


def get_tf_trajectory(obj: str, start_timestamp: float, end_timestamp: float) -> List:
    res = knowrob_client.once(f"tf_mng_trajectory({atom(obj)}, {start_timestamp}, {end_timestamp}, Trajectory)")
    return res["Trajectory"]


def get_wrench_trajectory(obj: str, start_timestamp: float, end_timestamp: float) -> List:
    res = knowrob_client.once(f"wrench_mng_trajectory({atom(obj)}, {start_timestamp}, {end_timestamp}, Trajectory)")
    return res["Trajectory"]


# generic workaround
def triple(subject, predicate, obj):
    rospy.loginfo(f"Adding triple: {subject}, {predicate}, {obj}")
    res = knowrob_client.once(f"triple({atom(subject)}, {atom(predicate)}, {atom(obj)}).")
    return res


def make_instance_of(class_iri):
    res = knowrob_client.once(
        f"kb_project([new_iri(Instance, {atom(class_iri)}), has_type(Instance, {atom(class_iri)})])")
    return res["Instance"]


def add_pose_to_instance(instance, pose_array):  # instance of Location
    res = knowrob_client.once(f"kb_project(([new_iri(PoseObj, soma:'6DPose'), has_type(PoseObj, soma:'6DPose'),"
                                  f"triple({atom(instance)}, 'http://www.ease-crc.org/ont/SOMA.owl#hasLocation', PoseObj)])),"
                                  f"time_scope({rospy.rostime.get_time()}, {rospy.rostime.get_time()}, Scope),"
                                  f"tf_set_pose(PoseObj, {pose_array}, Scope).")
    return res


def add_object_designator_description(object_designator_description):
    # TODO ensure existance ob obj_designator_name
    # ensure name exists
    if len(object_designator_description.names[0]) > 0:
        name = object_designator_description.names[0]
        query_part = f"triple(PhysicalObject, soma:'hasNameString', {atom(name)})"

    elif len(object_designator_description.types[0]) > 0:
        type = object_designator_description.types[0]  # TODO might need matching from Enum to String or smth.
        query_part = f"triple(PhysicalObject, dul:'classifies', {atom(type)})"

    else:
        query_part = ""

    res = knowrob_client.once(
        f"kb_project([new_iri(ObjectDesigDesc, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Object_Designator_Design'), "
        f"has_type(ObjectDesigDesc, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Object_Designator_Design'),"
        f"new_iri(PhysicalObject, dul:'PhysicalObject'), has_type(PhysicalObject, dul:'PhysicalObject'),"
        f"triple(ObjectDesigDesc, dul:'describes', PhysicalObject),"
        f"{query_part}]),"
        f"instance_of(ObjectDesigDesc, Class).")
    return res["ObjectDesigDesc"]  #


# an information object in this case is an IRI of an object designator description
def add_information_object(information_object):
    res = knowrob_client.once(f"kb_project([new_iri(InformationObject, dul:'Information_Object'), "
                                  f"has_type(InformationObject, dul:'Information_Object'),"
                                  f"triple(InformationObject, dul:'isExpressedBy', {atom(information_object)})]),"
                                  f"instance_of(InformationObject, Class).")
    return res["InformationObject"]


# todo: take into account if params are not object designators
def add_location_designator_description(dict_of_object_designator_descriptions):  # WIP
    # todo this should work without hardcoded field names
    collected_items = {}
    query_part = ""
    furniture_item, room = None, None
    if dict_of_object_designator_descriptions.get("furniture_item"):
        furniture_item = dict_of_object_designator_descriptions.get("furniture_item")
        information_furniture_item = self.add_information_object(furniture_item)
        query_part += f", triple(LocationDesigDesc, dul:'isExpressedBy', {atom(information_furniture_item)})"

    if dict_of_object_designator_descriptions.get("room"):
        room = dict_of_object_designator_descriptions.get("room")
        information_room = self.add_information_object(room)
        query_part += f", triple(LocationDesigDesc, dul:'isExpressedBy', {atom(information_room)})"

    if furniture_item and room:
        query_part += f", triple({atom(furniture_item)}, soma:'isInsideOf', {atom(room)})"
    # build query parts of the parameters given above
    res = knowrob_client.once(
        f"kb_project([new_iri(LocationDesigDesc, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Location_Designator_Design'), "
        f"has_type(LocationDesigDesc, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Location_Designator_Design'),"
        f"new_iri(Location, soma:'Location'), has_type(Location, soma:'Location'),"
        f"triple(LocationDesigDesc, 'http://www.ease-crc.org/ont/SOMA.owl#hasLocation', Location)"
        f"{query_part}]),"
        f"instance_of(LocationDesigDesc, Class)."
        )
    return res["LocationDesigDesc"]


def add_resolved_location_designator(resolved_location_designator, location_designator_description_iri):
    all_possible_poses = resolved_location_designator.poses
    all_poses_array = []
    all_poses_query_part = ", "
    # todo: log all possible poses as the outcome of designator resolution
    for pose in all_possible_poses:
        pose_array = [pose.frame, pose.position_as_list(), pose.orientation_as_list()]
        all_poses_array.append(pose_array)
        all_poses_query_part += f"kb_project(([new_iri(PoseObj, soma:'6DPose'), has_type(PoseObj, soma:'6DPose')," \
                                f"triple(Location, soma:'hasLocation', PoseObj)]))," \
                                f"time_scope({rospy.rostime.get_time()}, {rospy.rostime.get_time()}, Scope)," \
                                f"tf_set_pose(PoseObj, {pose_array}, Scope),"

    # add_pose_to_instance(loc_inst, pose_array)
    res = knowrob_client.once(
        f"kb_project([new_iri(ResolvedLocationDesig, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Location_Designator'), "
        f"has_type(ResolvedLocationDesig, 'http://www.ease-crc.org/ont/SOMA-CRAM.owl#Location_Designator'),"
        f"triple(ResolvedLocationDesig, dul:'expresses', {atom(location_designator_description_iri)}),"
        f"triple({atom(location_designator_description_iri)}, soma_cram:'resolved_to', ResolvedLocationDesig),"
        f"new_iri(Location, soma:'Location'), has_type(Location, soma:'Location')]),"
        f"instance_of(Location, LocClass)"
        f"{all_poses_query_part} "
        f"instance_of(ResolvedLocationDesig, Class)."
        )
    # todo: fix return result to be the ID of the Location Designator
    return res["ResolvedLocationDesig"]


# class Episode:
#     """
#     Convenience object and context manager for NEEM creation. Can be used in a 'with' statement to automatically
#     start and end a NEEM context (episode).
#     """
#
#     def __init__(neem_interface: NEEMInterface, task_type: str, env_owl: str, env_owl_ind_name: str,
#                  env_urdf: str, env_urdf_prefix: str, agent_owl: str, agent_owl_ind_name: str, agent_urdf: str,
#                  neem_output_path: str,
#                  start_time=None):
#         self.neem_interface = neem_interface
#         self.task_type = task_type
#         self.env_owl = env_owl
#         self.env_owl_ind_name = env_owl_ind_name
#         self.env_urdf = env_urdf
#         self.env_urdf_prefix = env_urdf_prefix
#         self.agent_owl = agent_owl
#         self.agent_owl_ind_name = agent_owl_ind_name
#         self.agent_urdf = agent_urdf
#         self.neem_output_path = neem_output_path
#
#         self.top_level_action_iri = None
#         self.episode_iri = None
#         self.start_time = start_time if start_time is not None else time.time()
#
#     def __enter__(self):
#         self.top_level_action_iri = self.neem_interface.start_episode(self.task_type, self.env_owl,
#                                                                       self.env_owl_ind_name, self.env_urdf,
#                                                                       self.env_urdf_prefix,
#                                                                       self.agent_owl, self.agent_owl_ind_name,
#                                                                       self.agent_urdf,
#                                                                       self.start_time)
#         self.episode_iri = \
#             self.neem_interface.prolog.once(
#                 f"kb_call(is_setting_for(Episode, {atom(self.top_level_action_iri)}))")[
#                 "Episode"]
#         return self
#
#     def __exit__(exc_type, exc_val, exc_tb):
#         self.neem_interface.stop_episode(self.neem_output_path)
