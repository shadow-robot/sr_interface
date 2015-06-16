#!/usr/bin/env python

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("basic_arm_examples", anonymous=True)

hand_commander = SrHandCommander(name="left_hand", prefix="lh")
arm_commander = SrArmCommander(name="left_arm", set_ground=False)

hand_joint_states_1 = {'lh_RFJ2': 1.5872158005941464, 'lh_RFJ3': 1.4895168741913918, 'lh_RFJ1': 1.4666237408137237,
                       'lh_RFJ4': -0.013050215160411582, 'lh_LFJ4': 0.01625614455626714, 'lh_LFJ5': 0.06044670177162791,
                       'lh_LFJ1': 1.407234521173739, 'lh_LFJ2': 1.6004770713418932, 'lh_LFJ3': 1.4891552968918866,
                       'lh_THJ2': 0.6375448576161045, 'lh_THJ3': -0.08707199480655038, 'lh_THJ1': 0.4292303516130266,
                       'lh_THJ4': 0.4910109864772066, 'lh_THJ5': 0.3494250743459288, 'lh_FFJ4': -0.023940821288538697,
                       'lh_FFJ2': 1.705298764563267, 'lh_FFJ3': 1.486354199529797, 'lh_FFJ1': 1.2470199410677854,
                       'lh_MFJ3': 1.486614783269636, 'lh_MFJ2': 1.6564932487003736, 'lh_MFJ1': 1.3140315426072693,
                       'lh_MFJ4': -0.0242471489068632, 'lh_WRJ2': -0.02486374414999284, 'lh_WRJ1': 0.029153388390064056}
hand_joint_states_2 = {'lh_RFJ2': 0.5478785575312844, 'lh_RFJ3': 0.08445184105173437, 'lh_RFJ1': 0.029420452374822992,
                       'lh_RFJ4': -0.15015641393093462, 'lh_LFJ4': -0.3537140525902314, 'lh_LFJ5': 0.23522928790906183,
                       'lh_LFJ1': 0.016491300018844024, 'lh_LFJ2': 0.49407520333290567,
                       'lh_LFJ3': -0.020286553857819023, 'lh_THJ2': -0.08248675629081387,
                       'lh_THJ3': -0.0848751638665582, 'lh_THJ1': 0.15119341005233636, 'lh_THJ4': 0.555946758562008,
                       'lh_THJ5': -0.16597727036313847, 'lh_FFJ4': -0.33742084583325016, 'lh_FFJ2': 0.29498522568918245,
                       'lh_FFJ3': 0.15909058345101856, 'lh_FFJ1': 0.008391006019203506, 'lh_MFJ3': 0.19261624130519672,
                       'lh_MFJ2': 0.4993988321882934, 'lh_MFJ1': 0.0018180513041607393, 'lh_MFJ4': -0.06813011563236507,
                       'lh_WRJ2': -0.029337204975412538, 'lh_WRJ1': -0.024214584973550318}
hand_joint_states_3 = {'lh_RFJ2': 0.6281074021794109, 'lh_RFJ3': 0.7659888823679098, 'lh_RFJ1': 0.03325790268458251,
                       'lh_RFJ4': -0.01845843339542446, 'lh_LFJ4': -0.3151306683738226, 'lh_LFJ5': 0.6682392109822538,
                       'lh_LFJ1': 0.02267553752591056, 'lh_LFJ2': 0.7320423099057738, 'lh_LFJ3': 0.21083959411165956,
                       'lh_THJ2': -0.05850835753848048, 'lh_THJ3': -0.03831702846125888, 'lh_THJ1': 0.3880903237967719,
                       'lh_THJ4': 0.8542756038519191, 'lh_THJ5': 0.3977832749115066, 'lh_FFJ4': -0.3461906922375515,
                       'lh_FFJ2': 0.9047110422867736, 'lh_FFJ3': 0.5607576049255242, 'lh_FFJ1': 0.020697814847368656,
                       'lh_MFJ3': 0.5934438572029458, 'lh_MFJ2': 0.8413647783663876, 'lh_MFJ1': 0.05045092369046106,
                       'lh_MFJ4': -0.0763291870248651, 'lh_WRJ2': -0.03623944626996376, 'lh_WRJ1': 0.15273292976946237}
hand_joint_states_4 = {'lh_RFJ2': 0.5689914113860546, 'lh_RFJ3': 0.2699064481161507, 'lh_RFJ1': 0.044770253613861055,
                       'lh_RFJ4': -0.01310439794863375, 'lh_LFJ4': -0.27636350128170994, 'lh_LFJ5': 0.3923167588877166,
                       'lh_LFJ1': 0.005668884381477612, 'lh_LFJ2': 0.7213711392074389, 'lh_LFJ3': -0.12022805250412126,
                       'lh_THJ2': -0.19285193046975369, 'lh_THJ3': -0.049175771502373795, 'lh_THJ1': 0.3772016453548229,
                       'lh_THJ4': 0.851042148000176, 'lh_THJ5': -0.10598914088076919, 'lh_FFJ4': -0.32111300494638306,
                       'lh_FFJ2': 0.6403305026287431, 'lh_FFJ3': -0.033504455579867615, 'lh_FFJ1': 0.03636102608321523,
                       'lh_MFJ3': 0.03648567566570239, 'lh_MFJ2': 0.8329697861210468, 'lh_MFJ1': 0.009090256520803808,
                       'lh_MFJ4': -0.05363466958654512, 'lh_WRJ2': -0.02912809841748016, 'lh_WRJ1': 0.15505134454423647}
hand_joint_states_5 = {'lh_RFJ2': 1.583110932144334, 'lh_RFJ3': 1.5194606949913108, 'lh_RFJ1': 1.3397225906184769,
                       'lh_RFJ4': -0.06020330131439302, 'lh_LFJ4': -0.19643249787043804, 'lh_LFJ5': 0.09445042306544366,
                       'lh_LFJ1': 1.4705487685109613, 'lh_LFJ2': 1.5685131925989737, 'lh_LFJ3': 1.395806825047366,
                       'lh_THJ2': -0.0076095150843690645, 'lh_THJ3': -0.04107203065574469,
                       'lh_THJ1': 0.29468156960530606, 'lh_THJ4': 0.5994495871408213, 'lh_THJ5': -1.3629363677138682,
                       'lh_FFJ4': 0.027566020957249734, 'lh_FFJ2': 1.7233124839072451, 'lh_FFJ3': 1.407416123169923,
                       'lh_FFJ1': 1.2125585930819789, 'lh_MFJ3': 1.399505129145413, 'lh_MFJ2': 1.6508429022011113,
                       'lh_MFJ1': 1.3258988225487143, 'lh_MFJ4': 0.1162845586275759, 'lh_WRJ2': -0.04645869454675692,
                       'lh_WRJ1': 0.16639311705105272}

arm_joint_states_1 = {'la_shoulder_lift_joint': -1.8675444761859339, 'la_elbow_joint': 1.762739658355713,
                      'la_wrist_2_joint': 0.034899186342954636, 'la_wrist_1_joint': -0.855201546345846,
                      'la_shoulder_pan_joint': -2.6354027430163782, 'la_wrist_3_joint': 0.6980043649673462}
arm_joint_states_2 = {'la_shoulder_lift_joint': -1.858894173298971, 'la_elbow_joint': 1.8534269332885742,
                      'la_wrist_2_joint': -0.1952908674823206, 'la_wrist_1_joint': -0.9622061888324183,
                      'la_shoulder_pan_joint': -1.7790520826922815, 'la_wrist_3_joint': 1.0615997314453125}
arm_joint_states_3 = {'la_shoulder_lift_joint': -1.8589418570147913, 'la_elbow_joint': 1.9028244018554688,
                      'la_wrist_2_joint': -0.17748672166933233, 'la_wrist_1_joint': -0.9611042181598108,
                      'la_shoulder_pan_joint': -1.7789681593524378, 'la_wrist_3_joint': 1.0615878105163574}
arm_joint_states_4 = {'la_shoulder_lift_joint': -1.3324082533465784, 'la_elbow_joint': 1.1092381477355957,
                      'la_wrist_2_joint': 1.0047885179519653, 'la_wrist_1_joint': 0.12960267066955566,
                      'la_shoulder_pan_joint': -1.493253533040182, 'la_wrist_3_joint': 3.2672297954559326}
arm_joint_states_5 = {'la_shoulder_lift_joint': -1.4488886992083948, 'la_elbow_joint': 1.1080164909362793,
                      'la_wrist_2_joint': 0.8986303806304932, 'la_wrist_1_joint': 0.45307910442352295,
                      'la_shoulder_pan_joint': -0.9496935049640101, 'la_wrist_3_joint': 0.09605655819177628}
arm_joint_states_6 = {'la_shoulder_lift_joint': -1.3556659857379358, 'la_elbow_joint': 1.16654634475708,
                      'la_wrist_2_joint': 0.9602982401847839, 'la_wrist_1_joint': 0.39066922664642334,
                      'la_shoulder_pan_joint': -0.9169490973102015, 'la_wrist_3_joint': 0.09618838876485825}
arm_joint_states_7 = {'la_shoulder_lift_joint': -1.3484209219561976, 'la_elbow_joint': 1.0440430641174316,
                      'la_wrist_2_joint': 1.5493569374084473, 'la_wrist_1_joint': 0.08041524887084961,
                      'la_shoulder_pan_joint': -1.6396897474872034, 'la_wrist_3_joint': -1.407604996358053}
arm_joint_states_8 = {'la_shoulder_lift_joint': -1.5496943632708948, 'la_elbow_joint': 1.4070467948913574,
                      'la_wrist_2_joint': 0.023541511967778206, 'la_wrist_1_joint': 0.6106277704238892,
                      'la_shoulder_pan_joint': -1.549493137990133, 'la_wrist_3_joint': -0.5691126028644007}
# Move hand
joint_states = hand_joint_states_1
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, False)
# Move arm
joint_states = arm_joint_states_1
rospy.loginfo("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)

# Move hand
joint_states = hand_joint_states_2
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, False)
# Move arm
joint_states = arm_joint_states_2
rospy.loginfo("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)

# Move arm
joint_states = arm_joint_states_3
rospy.loginfo("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 1.0, True)
# Move hand
joint_states = hand_joint_states_3
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True)

# Move arm
# joint_states = arm_joint_states_4
# rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)

# Move arm
# joint_states = arm_joint_states_5
# rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
# arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)

# Move arm
joint_states = arm_joint_states_6
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)

# Move hand
joint_states = hand_joint_states_4
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True)

# Move arm
joint_states = arm_joint_states_5
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 2.0, True)

# Move arm
joint_states = arm_joint_states_7
rospy.loginfo("Moving arm to joint states\n" + str(joint_states) + "\n")
arm_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, False)
# Move hand
joint_states = hand_joint_states_5
rospy.loginfo("Moving hand to joint states\n" + str(joint_states) + "\n")
hand_commander.move_to_joint_value_target_unsafe(joint_states, 3.0, True)
