
#Этот файл автоматически сгенерирован с помощью программы IronArt v0.101
#Не следует его редактировать вручную

class roki2met :
    #Индексы доступных слотов
    mixingRoki2metSlot = 1
    mixingRoki2metSlot = 2
    metalSitSlot = 3
    metalHandSlot = 4
    metalShiftSlot = 5
    metalWalkSlot = 6
    metalTestSlot = 7
    TripleJumpSlot = 8
    ArcherySlot = 9
    ArcheryShootSlot = 10
    BasketballSlot = 11
    basketballSecondSlot = 12
    OpenClawSlot = 13
    weight1Slot = 14
    weight2Slot = 15
    basketballFirstSlot = 16
    ArcheryDropSlot = 17
    sprint_v1Slot = 19
    sprint_v2Slot = 20
    sprint_v3Slot = 21
    sprint_v4Slot = 22
    roki2WalkStraight_v1Slot = 25



    #Переменные, присутствующие во всех слотах
    svFrameCount = 1
    svImuAccX = 1000
    svImuAccY = 1001
    svImuAccZ = 1002
    svImuAccRawX = 1003
    svImuAccRawY = 1004
    svImuAccRawZ = 1005
    svImuGyroX = 1006
    svImuGyroY = 1007
    svImuGyroZ = 1008
    svImuGyroRawX = 1009
    svImuGyroRawY = 1010
    svImuGyroRawZ = 1011
    svImuQuaterX = 1012
    svImuQuaterY = 1013
    svImuQuaterZ = 1014
    svImuQuaterW = 1015
    svImuQuaterAcc = 1016
    svImuAccSlowX = 1017
    svImuAccSlowY = 1018
    svImuAccSlowZ = 1019
    svAdc = 1020
    svButtonLeft = 1021
    svButtonRight = 1022
    svHeadControlMask = 1023
    svRemoteStatus = 1190
    svRemoteButton = 1191
    svRemoteLeftJoystick = 1192
    svRemoteLeftJoystickX = 1193
    svRemoteLeftJoystickY = 1194
    svRemoteRightJoystick = 1195
    svRemoteRightJoystickX = 1196
    svRemoteRightJoystickY = 1197
    svHumoModelDistPelvic = 1204
    svHumoModelLengthPelvicHip = 1205
    svHumoModelLengthHipHip = 1206
    svHumoModelLengthHipKnee = 1207
    svHumoModelLengthKneeFoot = 1208
    svHumoModelLengthFootFoot = 1209
    svHumoModelDistFootFloor = 1210
    svHumoModelFootLength = 1211
    svHumoModelFootWidth = 1212
    svHumoModelFootCenterBack = 1213
    svHumoModelFootCenterInner = 1214
    svHumoModelAngPelvic = 1198
    svHumoModelAngHipSide = 1200
    svHumoModelAngHip = 1199
    svHumoModelAngKnee = 1201
    svHumoModelAngFoot = 1202
    svHumoModelAngFootSide = 1203
    svIkOutPresent = 1237
    svIkEncA5 = 1238
    svIkEncA6 = 1239
    svIkEncA7 = 1240
    svIkEncA8 = 1241
    svIkEncA9 = 1242
    svIkEncA10 = 1243
    headRotateTarget = 1030
    headRotateCurrent = 1031
    headRotatePwm = 1032
    headRotateAddonMix = 1033
    torsoRotateTarget = 1034
    torsoRotateCurrent = 1035
    torsoRotatePwm = 1036
    torsoRotateAddonMix = 1037
    rightClavicleTarget = 1038
    rightClavicleCurrent = 1039
    rightClaviclePwm = 1040
    rightClavicleAddonMix = 1041
    leftClavicleTarget = 1042
    leftClavicleCurrent = 1043
    leftClaviclePwm = 1044
    leftClavicleAddonMix = 1045
    rightShoulderTarget = 1046
    rightShoulderCurrent = 1047
    rightShoulderPwm = 1048
    rightShoulderAddonMix = 1049
    leftShoulderTarget = 1050
    leftShoulderCurrent = 1051
    leftShoulderPwm = 1052
    leftShoulderAddonMix = 1053
    rightElbowSideTarget = 1054
    rightElbowSideCurrent = 1055
    rightElbowSidePwm = 1056
    rightElbowSideAddonMix = 1057
    leftElbowSideTarget = 1058
    leftElbowSideCurrent = 1059
    leftElbowSidePwm = 1060
    leftElbowSideAddonMix = 1061
    rightElbowTarget = 1062
    rightElbowCurrent = 1063
    rightElbowPwm = 1064
    rightElbowAddonMix = 1065
    leftElbowTarget = 1066
    leftElbowCurrent = 1067
    leftElbowPwm = 1068
    leftElbowAddonMix = 1069
    rightPelvicTarget = 1070
    rightPelvicCurrent = 1071
    rightPelvicPwm = 1072
    rightPelvicAddonMix = 1073
    leftPelvicTarget = 1074
    leftPelvicCurrent = 1075
    leftPelvicPwm = 1076
    leftPelvicAddonMix = 1077
    rightHipSideTarget = 1078
    rightHipSideCurrent = 1079
    rightHipSidePwm = 1080
    rightHipSideAddonMix = 1081
    leftHipSideTarget = 1082
    leftHipSideCurrent = 1083
    leftHipSidePwm = 1084
    leftHipSideAddonMix = 1085
    rightHipTarget = 1086
    rightHipCurrent = 1087
    rightHipPwm = 1088
    rightHipAddonMix = 1089
    leftHipTarget = 1090
    leftHipCurrent = 1091
    leftHipPwm = 1092
    leftHipAddonMix = 1093
    rightKneeTarget = 1094
    rightKneeCurrent = 1095
    rightKneePwm = 1096
    rightKneeAddonMix = 1097
    leftKneeTarget = 1098
    leftKneeCurrent = 1099
    leftKneePwm = 1100
    leftKneeAddonMix = 1101
    rightFootFrontTarget = 1102
    rightFootFrontCurrent = 1103
    rightFootFrontPwm = 1104
    rightFootFrontAddonMix = 1105
    leftFootFrontTarget = 1106
    leftFootFrontCurrent = 1107
    leftFootFrontPwm = 1108
    leftFootFrontAddonMix = 1109
    rightFootSideTarget = 1110
    rightFootSideCurrent = 1111
    rightFootSidePwm = 1112
    rightFootSideAddonMix = 1113
    leftFootSideTarget = 1114
    leftFootSideCurrent = 1115
    leftFootSidePwm = 1116
    leftFootSideAddonMix = 1117
    rightHandTarget = 1118
    rightHandCurrent = 1119
    rightHandPwm = 1120
    rightHandAddonMix = 1121
    leftHandTarget = 1122
    leftHandCurrent = 1123
    leftHandPwm = 1124
    leftHandAddonMix = 1125
    headTiltTarget = 1130
    headTiltCurrent = 1131
    headTiltPwm = 1132
    headTiltAddonMix = 1133
    rightKneeBotTarget = 1134
    rightKneeBotCurrent = 1135
    rightKneeBotPwm = 1136
    rightKneeBotAddonMix = 1137
    leftKneeBotTarget = 1138
    leftKneeBotCurrent = 1139
    leftKneeBotPwm = 1140
    leftKneeBotAddonMix = 1141
    rightClawTarget = 1142
    rightClawCurrent = 1143
    rightClawPwm = 1144
    rightClawAddonMix = 1145
    leftClawTarget = 1146
    leftClawCurrent = 1147
    leftClawPwm = 1148
    leftClawAddonMix = 1149



    #Классы слотов с индексами переменных, определенных в этих слотах


    class mixingRoki2met :
        #Перечень переменных слота mixingRoki2met
        leftRightFactor = 2
        frontBackFactor = 3
        quart = 4
        errCount = 5


    class mixingRoki2met :
        #Перечень переменных слота mixingRoki2met
        leftRightFactor = 2
        frontBackFactor = 3
        quart = 4
        errCount = 5


    class metalSit :
        #Перечень переменных слота metalSit
        angle = 2
        velo = 3
        factor = 4
        frames = 5


    class metalHand :
        #Перечень переменных слота metalHand
        angle = 2
        velo = 3
        factor = 4
        frames = 5


    class metalShift :
        #Перечень переменных слота metalShift
        walkLeft = 2
        walkStandAngle = 3
        walkSidingAngle = 4
        walkFootAddon = 5
        walkLegUpAngle = 6
        walkTorsoAngle = 7
        walkLegForward = 8
        walkHipBackward = 9
        walkLegDnAngle = 10
        walkMoveSide = 11
        walkMoveRotate = 12
        walkMoveForward = 13
        frame = 14
        angle = 15
        angleFoot = 16
        maskSingleAngle = 17
        maskDoubleAngle = 18
        pushFrame = 19
        slideFrame = 20
        legUpFrame = 21
        downFrame = 22
        downSideFrame = 23


    class metalWalk :
        #Перечень переменных слота metalWalk
        walkLeft = 2
        walkSign = 3
        walkBaseHeight = 4
        walkLegUp = 5
        walkPush = 6
        imuXOffset = 7
        walkFront = 8
        walkBack = 9
        tiltForward = 10
        walkType = 11
        drawdown = 12
        forwardFactor = 13
        forwardPelvic = 14
        tiltAddon = 15
        flag = 16
        veloForward = 17
        isWalk = 18
        deltaFront = 19
        frontStep = 20
        heightStep = 21
        index = 22
        posFront = 23
        posHeight = 24

    class sprint_v1 :
        #Перечень переменных слота sprint_v1
        slowWalk = 2
        selfFirstLegIsRightLeg = 15
        motion_to_right = 16
        selfInitPoses = 18
        selfExitFlag = 19
        selfFallingFlag = 20
        selfNeckPan = 21
        framestep = 24
        fr1 = 43
        fr2 = 44
        stepType = 57
        fps = 58
        timeStep = 59
        stepNumber = 62
        flag = 63
        flag_event = 64
        j = 65
        leftRightFactor = 78
        frontBackFactor = 79


    class sprint_v2 :
        #Перечень переменных слота sprint_v2
        slowWalk = 2
        selfFirstLegIsRightLeg = 15
        motion_to_right = 16
        selfInitPoses = 18
        selfExitFlag = 19
        selfFallingFlag = 20
        selfNeckPan = 21
        framestep = 24
        fr1 = 43
        fr2 = 44
        stepType = 57
        fps = 58
        timeStep = 59
        stepNumber = 62
        flag = 63
        flag_event = 64
        j = 65
        leftRightFactor = 78
        frontBackFactor = 79


    class sprint_v3 :
        #Перечень переменных слота sprint_v3
        slowWalk = 2
        selfFirstLegIsRightLeg = 13
        motion_to_right = 14
        selfInitPoses = 16
        selfExitFlag = 17
        selfFallingFlag = 18
        selfNeckPan = 19
        framestep = 22
        fr1 = 41
        fr2 = 42
        stepType = 55
        fps = 56
        timeStep = 57
        hipTilt = 60
        stepNumber = 61
        flag = 62
        flag_event = 63
        torsoAdd = 67
        j = 68
        leftRightFactor = 86
        frontBackFactor = 87


    class Sprint_v4 :
        #Перечень переменных слота sprint_v4
        slowWalk = 2
        pitStop = 3
        selfFirstLegIsRightLeg = 14
        motion_to_right = 15
        selfInitPoses = 17
        selfExitFlag = 18
        selfFallingFlag = 19
        selfNeckPan = 20
        framestep = 23
        fr1 = 42
        fr2 = 43
        stepType = 56
        fps = 57
        timeStep = 58
        hipTilt = 61
        stepNumber = 62
        flag = 63
        flag_event = 64
        torsoAdd = 68
        j = 71
        leftRightFactor = 87
        frontBackFactor = 88


    class roki2WalkStraight_v1 :
        #Перечень переменных слота roki2WalkStraight_v1
        slowWalk = 2
        selfFirstLegIsRightLeg = 12
        selfInitPoses = 13
        selfExitFlag = 14
        selfFallingFlag = 15
        selfNeckPan = 16
        framestep = 19
        fr1 = 38
        fr2 = 39
        stepType = 52
        fps = 53
        frames_per_cycle = 54
        relaxFootR = 55
        relaxFootL = 56
        segments_num = 59
        segments_sum = 160
        j = 161
        i = 162
        leftRightFactor = 194
        frontBackFactor = 195
