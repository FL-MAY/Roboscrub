# Type defines the mission type
# ID is a key in database for a more detailed mission description 
# Candidate Action type
uint8 AUTOMATIC     = 1  # Automatic mode
uint8 MANUAL        = 2  # Manual mode
uint8 MAPPING       = 3  # Generate map
uint8 TRACKING      = 4  # Manually draw a path
uint8 CHARGING      = 5  # Charging
uint8 UPDATING	    = 6  # Updating  	
#end Actions 
uint8 type
uint32 id
