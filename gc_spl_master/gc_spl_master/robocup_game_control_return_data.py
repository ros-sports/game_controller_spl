from construct import Array, Byte, Const, Default, Float32l, Struct

GAMECONTROLLER_RETURN_PORT = 3939

GAMECONTROLLER_RETURN_STRUCT_HEADER = b'RGrt'
GAMECONTROLLER_RETURN_STRUCT_VERSION = 4

RoboCupGameControlReturnData = Struct(
    'header' / Const(GAMECONTROLLER_RETURN_STRUCT_HEADER),  # "RGrt"
    'version' / Const(GAMECONTROLLER_RETURN_STRUCT_VERSION, Byte),  # has to be set to GAMECONTROLLER_RETURN_STRUCT_VERSION
    'playerNum' / Default(Byte, 0),  # player number starts with 1
    'teamNum' / Default(Byte, 0),  # team number
    'fallen' / Default(Byte, 255),  # 1 means that the robot is fallen, 0 means that the robot can play
    # position and orientation of robot
    # coordinates in millimeters
    # 0,0 is in center of field
    # +ve x-axis points towards the goal we are attempting to score on
    # +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    # angle in radians, 0 along the +x axis, increasing counter clockwise
    'pose' / Default(Array(3, Float32l), [0, 0, 0]),  # x,y,theta
    # ball information
    # seconds since this robot last saw the ball. -1.f if we haven't seen it
    'ballAge' / Default(Float32l, -1),
    # position of ball relative to the robot
    # coordinates in millimeters
    # 0,0 is in center of the robot
    # +ve x-axis points forward from the robot
    # +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
    'ball' / Default(Array(2, Float32l), [0, 0])
)
