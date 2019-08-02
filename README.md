# Overview

It is a software that creates a traffic simulation environment with model-based traffic agents and with a user-controlled vehicle which we will call the ego vehicle.

**Core Features:**
 - Software can be connected to the gym environment. 
   - The ego vehicle could be trained with reinforcement-learning algorithms to survive in a traffic environment by making intelligent lane change movements.
 - Custom highway environment can be created by adjusting these values:
   - Number of lanes
   - Number of vehicles
   - Agression and politeness factors of traffic agents
   - Initial distances between vehicles
   - Traffic rules => UK rules or USA rules
