MAP = {
    # DEAD_END is converted to FILLED for filled spaces 
    # and COLLECTED for collected boxes. Once all boxes are 
    # collected, COLLECTED will be replaced by DEAD_END again
    
    # ==================================================
    # CENTRAL NETWORK
    # ==================================================

    "START": {
        "type": "FOUR_JUNCTION",
        "north": "TOP_CENTRE",
        "south": None,
        "east": "MIDDLE_RIGHT",
        "west": "JUNCTION_LEFT_BOTTOM",
    },

    "TOP_CENTRE": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_TOP_DROP_1",
        "south": "START",
        "east": "TOP_RIGHT",
        "west": "TOP_LEFT",
    },

    "BOTTOM_CENTRE": {
        "type": "FOUR_JUNCTION",
        "north": None,
        "south": None,
        "east": "BOTTOM_RIGHT",
        "west": "BOTTOM_LEFT",
    },

    "MIDDLE_LEFT": {
        "type": "T_JUNCTION",
        "north": None,
        "south": "BOTTOM_LEFT",
        "east": "START",
        "west": "JUNCTION_LEFT_BOTTOM",
    },

    "MIDDLE_RIGHT": {
        "type": "T_JUNCTION",
        "north": None,
        "south": "BOTTOM_RIGHT",
        "east": "JUNCTION_RIGHT_BOTTOM",
        "west": "START",
    },

    "TOP_LEFT": {
        "type": "CORNER",
        "north": None,
        "south": "MIDDLE_LEFT",
        "east": "TOP_CENTRE",
        "west": None,
    },

    "TOP_RIGHT": {
        "type": "CORNER",
        "north": None,
        "south": "MIDDLE_RIGHT",
        "east": None,
        "west": "TOP_CENTRE",
    },

    "BOTTOM_LEFT": {
        "type": "CORNER",
        "north": "MIDDLE_LEFT",
        "south": None,
        "east": "BOTTOM_CENTRE",
        "west": None,
    },

    "BOTTOM_RIGHT": {
        "type": "CORNER",
        "north": "MIDDLE_RIGHT",
        "south": None,
        "east": None,
        "west": "BOTTOM_CENTRE",
    },


    # ==================================================
    # LEFT-SIDE NETWORK AND GREEN DEAD_ENDS
    # ==================================================

    "JUNCTION_LEFT_BOTTOM": {
        "type": "CORNER",
        "north": "JUNCTION_GREEN_1",
        "south": None,
        "east": "MIDDLE_LEFT",
        "west": None,
    },

    "JUNCTION_GREEN_1": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_GREEN_2",
        "south": "JUNCTION_LEFT_BOTTOM",
        "east": "GREEN_1",
        "west": None,
    },

    "JUNCTION_GREEN_2": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_LEFT_BOTTOM_DROP_1",
        "south": "JUNCTION_GREEN_1",
        "east": "GREEN_2",
        "west": None,
    },

    "GREEN_1": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_GREEN_1",
        "colour": "green",
    },

    "GREEN_2": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_GREEN_2",
        "colour": "green",
    },


    # ==================================================
    # RIGHT-SIDE NETWORK AND RED DEAD_ENDS
    # ==================================================

    "JUNCTION_RIGHT_BOTTOM": {
        "type": "CORNER",
        "north": "JUNCTION_RED_1",
        "south": None,
        "east": None,
        "west": "MIDDLE_RIGHT",
    },

    "JUNCTION_RED_1": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_RED_2",
        "south": "JUNCTION_RIGHT_BOTTOM",
        "east": None,
        "west": "RED_1",
    },

    "JUNCTION_RED_2": {
        "type": "T_JUNCTION",
        "north": "RIGHT_BOTTOM_DROP_1",
        "south": "JUNCTION_RED_1",
        "east": None,
        "west": "RED_2",
    },

    "RED_1": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RED_1",
        "west": None,
        "colour": "red",
    },

    "RED_2": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RED_2",
        "west": None,
        "colour": "red",
    },


    # ==================================================
    # LOWER DROP DESTINATIONS LEFT
    # Numbering is bottom to top.
    # ==================================================

    "JUNCTION_LEFT_BOTTOM_DROP_1": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_LEFT_BOTTOM_DROP_2",
        "south": "JUNCTION_GREEN_2",
        "east": "LEFT_BOTTOM_DROP_1",
        "west": None,
    },
    
    "LEFT_BOTTOM_DROP_1": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_LEFT_BOTTOM_DROP_1",
    },

    "JUNCTION_LEFT_BOTTOM_DROP_2": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_LEFT_BOTTOM_DROP_3",
        "south": "JUNCTION_LEFT_BOTTOM_DROP_1",
        "east": "LEFT_BOTTOM_DROP_2",
        "west": None,
    },
    
    "LEFT_BOTTOM_DROP_2": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_LEFT_BOTTOM_DROP_2",
    },
    
    "JUNCTION_LEFT_BOTTOM_DROP_3": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_LEFT_BOTTOM_DROP_4",
        "south": "JUNCTION_LEFT_BOTTOM_DROP_2",
        "east": "LEFT_BOTTOM_DROP_3",
        "west": None,
    },
    
    "LEFT_BOTTOM_DROP_3": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_LEFT_BOTTOM_DROP_3",
    },
    "JUNCTION_LEFT_BOTTOM_DROP_4": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_LEFT_BOTTOM_DROP_5",
        "south": "JUNCTION_LEFT_BOTTOM_DROP_3",
        "east": "LEFT_BOTTOM_DROP_4",
        "west": None,
    },
    
    "LEFT_BOTTOM_DROP_4": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_LEFT_BOTTOM_DROP_4",
    },
    
    "JUNCTION_LEFT_BOTTOM_DROP_5": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_LEFT_BOTTOM_DROP_6",
        "south": "JUNCTION_LEFT_BOTTOM_DROP_4",
        "east": "LEFT_BOTTOM_DROP_5",
        "west": None,
    },
    
    "LEFT_BOTTOM_DROP_5": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_LEFT_BOTTOM_DROP_5",
    },
    
    "JUNCTION_LEFT_BOTTOM_DROP_6": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_STOP_LEFT_CENTRE",
        "south": "JUNCTION_LEFT_BOTTOM_DROP_5",
        "east": "LEFT_BOTTOM_DROP_6",
        "west": None,
    },
    
    "LEFT_BOTTOM_DROP_6": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_LEFT_BOTTOM_DROP_6",
    },
    
    # STOPPING CAP
    
    "JUNCTION_STOP_LEFT_CENTRE": {
        "type": "T_JUNCTION",
        "north": None,
        "south": "JUNCTION_LEFT_BOTTOM_DROP_6",
        "east": "JUNCTION_STOP_LEFT_RIGHT",
        "west": "JUNCTION_STOP_LEFT_LEFT",
    },
    
    "JUNCTION_STOP_LEFT_LEFT": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_STOP_LEFT_CENTRE",
        "west": None,
    },
    
    "JUNCTION_STOP_LEFT_RIGHT": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_STOP_LEFT_CENTRE",
    },
    
    
    
    # ==================================================
    # LOWER DROP DESTINATIONS RIGHT
    # Numbering is bottom to top.
    # ==================================================

    "JUNCTION_RIGHT_BOTTOM_DROP_1": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_RIGHT_BOTTOM_DROP_2",
        "south": "JUNCTION_GREEN_2",
        "east": None,
        "west": "RIGHT_BOTTOM_DROP_1",
    },
    
    "RIGHT_BOTTOM_DROP_1": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RIGHT_BOTTOM_DROP_1",
        "west": None,
    },

    "JUNCTION_RIGHT_BOTTOM_DROP_2": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_RIGHT_BOTTOM_DROP_3",
        "south": "JUNCTION_RIGHT_BOTTOM_DROP_1",
        "east": None,
        "west": "RIGHT_BOTTOM_DROP_2",
    },
    
    "RIGHT_BOTTOM_DROP_2": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RIGHT_BOTTOM_DROP_2",
        "west": None,
    },
    
    "JUNCTION_RIGHT_BOTTOM_DROP_3": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_RIGHT_BOTTOM_DROP_4",
        "south": "JUNCTION_RIGHT_BOTTOM_DROP_2",
        "east": None,
        "west": "RIGHT_BOTTOM_DROP_3",
    },
    
    "RIGHT_BOTTOM_DROP_3": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RIGHT_BOTTOM_DROP_3",
        "west": None,
    },
    "JUNCTION_RIGHT_BOTTOM_DROP_4": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_RIGHT_BOTTOM_DROP_5",
        "south": "JUNCTION_RIGHT_BOTTOM_DROP_3",
        "east": None,
        "west": "RIGHT_BOTTOM_DROP_4",
    },
    
    "RIGHT_BOTTOM_DROP_4": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RIGHT_BOTTOM_DROP_4",
        "west": None,
    },
    
    "JUNCTION_RIGHT_BOTTOM_DROP_5": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_RIGHT_BOTTOM_DROP_6",
        "south": "JUNCTION_RIGHT_BOTTOM_DROP_4",
        "east": None,
        "west": "RIGHT_BOTTOM_DROP_5",
    },
    
    "RIGHT_BOTTOM_DROP_5": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RIGHT_BOTTOM_DROP_5",
        "west": None,
    },
    
    "JUNCTION_RIGHT_BOTTOM_DROP_6": {
        "type": "T_JUNCTION",
        "north": "JUNCTION_STOP_RIGHT_CENTRE",
        "south": "JUNCTION_RIGHT_BOTTOM_DROP_5",
        "east": None,
        "west": "RIGHT_BOTTOM_DROP_6",
    },
    
    "RIGHT_BOTTOM_DROP_6": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_RIGHT_BOTTOM_DROP_6",
        "west": None,
    },
    
    # STOPPING CAP
    
    "JUNCTION_STOP_RIGHT_CENTRE": {
        "type": "T_JUNCTION",
        "north": None,
        "south": "JUNCTION_RIGHT_BOTTOM_DROP_6",
        "east": "JUNCTION_STOP_RIGHT_LEFT",
        "west": "JUNCTION_STOP_RIGHT_RIGHT",
    },
    
    "JUNCTION_STOP_RIGHT_LEFT": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_STOP_RIGHT_CENTRE",
        "west": None,
    },
    
    "JUNCTION_STOP_RIGHT_RIGHT": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_STOP_RIGHT_CENTRE",
    },


    # ==================================================
    # UPPER DROP DESTINATIONS
    # ==================================================

    "JUNCTION_TOP_DROP_1": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_TOP_DROP_2",
        "south": "TOP_CENTRE",
        "east": "RIGHT_TOP_DROP_1",
        "west": "LEFT_TOP_DROP_1",
    },
    
    "LEFT_TOP_DROP_1": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_TOP_DROP_1",
        "west": None,
    },
    
    "RIGHT_TOP_DROP_1": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_TOP_DROP_1",
    },

    "JUNCTION_TOP_DROP_2": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_TOP_DROP_3",
        "south": "JUNCTION_TOP_DROP_1",
        "east": "RIGHT_TOP_DROP_2",
        "west": "LEFT_TOP_DROP_2",
    },
    
    "LEFT_TOP_DROP_2": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_TOP_DROP_2",
        "west": None,
    },
    
    "RIGHT_TOP_DROP_2": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_TOP_DROP_2",
    },

    "JUNCTION_TOP_DROP_3": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_TOP_DROP_4",
        "south": "JUNCTION_TOP_DROP_2",
        "east": "RIGHT_TOP_DROP_3",
        "west": "LEFT_TOP_DROP_3",
    },
    
    "LEFT_TOP_DROP_3": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_TOP_DROP_3",
        "west": None,
    },
    
    "RIGHT_TOP_DROP_3": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_TOP_DROP_3",
    },
    
    "JUNCTION_TOP_DROP_4": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_TOP_DROP_5",
        "south": "JUNCTION_TOP_DROP_3",
        "east": "RIGHT_TOP_DROP_4",
        "west": "LEFT_TOP_DROP_4",
    },
    
    "LEFT_TOP_DROP_4": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_TOP_DROP_4",
        "west": None,
    },
    
    "RIGHT_TOP_DROP_4": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_TOP_DROP_4",
    },
    
    "JUNCTION_TOP_DROP_5": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_TOP_DROP_6",
        "south": "JUNCTION_TOP_DROP_4",
        "east": "RIGHT_TOP_DROP_5",
        "west": "LEFT_TOP_DROP_5",
    },
    
    "LEFT_TOP_DROP_5": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_TOP_DROP_5",
        "west": None,
    },
    
    "RIGHT_TOP_DROP_5": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_TOP_DROP_5",
    },
    
    "JUNCTION_TOP_DROP_6": {
        "type": "FOUR_JUNCTION",
        "north": "JUNCTION_STOP_RIGHT_CENTRE",
        "south": "JUNCTION_TOP_DROP_5",
        "east": "RIGHT_TOP_DROP_6",
        "west": "LEFT_TOP_DROP_6",
    },
    
    "LEFT_TOP_DROP_6": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_TOP_DROP_6",
        "west": None,
    },
    
    "RIGHT_TOP_DROP_6": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_TOP_DROP_6",
    },
    
    # STOPPING CAP
    
    "JUNCTION_STOP_TOP_CENTRE": {
        "type": "T_JUNCTION",
        "north": None,
        "south": "JUNCTION_TOP_DROP_6",
        "east": "JUNCTION_STOP_TOP_LEFT",
        "west": "JUNCTION_STOP_TOP_RIGHT",
    },
    
    "JUNCTION_STOP_TOP_LEFT": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": "JUNCTION_STOP_RIGHT_CENTRE",
        "west": None,
    },
    
    "JUNCTION_STOP_TOP_RIGHT": {
        "type": "DEAD_END",
        "north": None,
        "south": None,
        "east": None,
        "west": "JUNCTION_STOP_TOP_CENTRE",
    },
}