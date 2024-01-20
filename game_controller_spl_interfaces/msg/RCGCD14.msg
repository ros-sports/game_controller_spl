uint8 COMPETITION_PHASE_ROUNDROBIN=0
uint8 COMPETIION_PHASE_PLAYOFF=1

uint8 COMPETITION_TYPE_NORMAL=0
uint8 COMPETITION_TYPE_CHALLENGE_SHIELD=1
uint8 COMPETITION_TYPE_7V7=2
uint8 COMPETITION_TYPE_DYNAMIC_BALL_HANDLING=3

uint8 GAME_PHASE_NORMAL=0
uint8 GAME_PHASE_PENALTYSHOOT=1
uint8 GAME_PHASE_OVERTIME=2
uint8 GAME_PHASE_TIMEOUT=3

uint8 STATE_INITIAL=0
uint8 STATE_READY=1
uint8 STATE_SET=2
uint8 STATE_PLAYING=3
uint8 STATE_FINISHED=4

uint8 SET_PLAY_NONE=0
uint8 SET_PLAY_GOAL_KICK=1
uint8 SET_PLAY_PUSHING_FREE_KICK=2
uint8 SET_PLAY_CORNER_KICK=3
uint8 SET_PLAY_KICK_IN=4
uint8 SET_PLAY_PENALTY_KICK=5

uint8 packet_number
uint8 players_per_team
uint8 competition_phase
uint8 competition_type
uint8 game_phase
uint8 state
uint8 set_play
uint8 first_half
uint8 kicking_team
int16 secs_remaining
int16 secondary_time
game_controller_spl_interfaces/TeamInfo14[2] teams
