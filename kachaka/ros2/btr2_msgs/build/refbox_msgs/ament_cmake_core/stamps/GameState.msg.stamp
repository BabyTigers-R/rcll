# The GameState message is periodically sent via peer-to-peer to all
# robots as well as by client-server communication to connected
# controllers. The game time is restarted in the exploration and
# production phases, i.e. the time goes from 0 to 180 seconds in the
# exploration phase and from 0 to 900 seconds in the production phase.
#
# In the PRE_GAME and POST_GAME phases as well as in states other
# than RUNNING the robots must stand still immediately and all the
# time.
#
# The SetGameState and SetGamePhase messages can be used to request
# setting a new game state or phase respectively. They may only be
# sent by a controller. Be careful, setting a new phase will trigger
# re-initialization of that phase. To interrupt a game set the state
# to PAUSE, only modify the phase if you want to change
# irreversibly. It is illegal to send these messages from a robot. It
# will be detected by the refbox and considered a fraud attempt.

int8 STATE_INIT = 0
int8 STATE_WAIT_START = 1
int8 STATE_RUNNING = 2
int8 STATE_PAUSED = 3

int8 PHASE_PRE_GAME    =  0
int8 PHASE_SETUP       = 10
int8 PHASE_EXPLORATION = 20
int8 PHASE_PRODUCTION  = 30
int8 PHASE_POST_GAME   = 40

# Time in seconds since game start
refbox_msgs/Time game_time

# Current game state
int8 state
# Current game phase
int8 phase

# Awarded points, cyan
uint32 points_cyan
# Name of the currently playing team
string team_cyan

# Awarded points, magenta
uint32 points_magenta
# Name of the currently playing team
string team_magenta


uint32 field_height
uint32 field_width
bool field_mirrored

